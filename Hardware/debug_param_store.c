#include "debug_param_store.h"
#include "spi_flash.h"
#include <string.h>

/* 专用最后 8KB：0xFFE000..0xFFFFFF，其他 Flash 内容不动。
 * 两扇区各16页，追加写入；满后只擦不含最新有效记录的扇区。
 * CRC 覆盖版本、序号、数据；写入→回读→单独提交→再回读才报告保存成功。 */
#define BASE        (SPI_FLASH_SIZE - 2U * SPI_FLASH_SECTOR)
#define MAGIC       0x50434C49UL
#define VERSION     2U
#define COMMIT      0x434F4D54UL
#define DEBOUNCE_MS 2000U
typedef struct {
  uint32_t magic, version, sequence, size;
  DebugParamValues params;
  uint32_t crc, commit;
} Record;
typedef char RecordLayoutCheck[(sizeof(Record) == 88U) ? 1 : -1];
enum { IDLE, ERASE_WAIT, ERASE_CHECK, WRITE_BODY, BODY_WAIT, COMMIT_WAIT, FAILED };
static uint8_t state, has_saved, needs_upgrade;
static uint8_t page_buffer[SPI_FLASH_PAGE];
static uint32_t free_pages, sequence, active_page, target_page, check_page;
static uint32_t changed_tick, operation_tick;
static DebugParamValues observed, saved;
static Record pending, readback;

static uint32_t Crc(const void *data, uint32_t length)
{
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFUL, i;
  while (length--) {
    crc ^= *p++;
    for (i = 0U; i < 8U; ++i) crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
  }
  return ~crc;
}

uint8_t DebugParamStore_Valid(const DebugParamValues *v)
{
  static const float low[16] = {0,0,0,0,0,0,0,-500,-500,1,1,-12.5f,-30,0,0,-10};
  static const float high[16] = {50,50,50,3000,3000,100,100,500,500,1791,2,12.5f,30,500,5,10};
  uint32_t i;
  for (i = 0U; i < 16U; ++i)
    if (!(v->value[i] >= low[i] && v->value[i] <= high[i])) return 0U; /* 同时拒绝 NaN/Inf */
  return (uint8_t)(v->value[9] == (float)(uint16_t)v->value[9] &&
                   v->value[10] == (float)(uint8_t)v->value[10]);
}

static uint8_t ValidRecord(const Record *r)
{
  return (uint8_t)(r->magic == MAGIC && (r->version == 1U || r->version == VERSION) && r->size == sizeof(DebugParamValues) &&
    r->commit == COMMIT && r->crc == Crc(r, 80U) && DebugParamStore_Valid(&r->params));
}

static uint8_t Blank(void)
{
  uint32_t i;
  for (i = 0U; i < sizeof(page_buffer); ++i) if (page_buffer[i] != 0xFFU) return 0U;
  return 1U;
}

ParamStoreResult DebugParamStore_Init(DebugParamValues *values)
{
  uint32_t i, loaded_version = VERSION;
  state = FAILED; has_saved = 0U; needs_upgrade = 0U;
  free_pages = 0U; sequence = 0U; active_page = 0U;
  if (SPIFlash_Init() != HAL_OK) return PARAM_STORE_ERROR;
  for (i = 0U; i < 32U; ++i) {
    if (SPIFlash_Read(BASE + i * SPI_FLASH_PAGE, page_buffer, SPI_FLASH_PAGE) != HAL_OK)
      return PARAM_STORE_ERROR;
    if (Blank()) free_pages |= (1UL << i);
    memcpy(&readback, page_buffer, sizeof(readback));
    if (ValidRecord(&readback) && (!has_saved ||
        (int32_t)(readback.sequence - sequence) > 0)) {
      saved = readback.params; sequence = readback.sequence; active_page = i; has_saved = 1U;
      loaded_version = readback.version;
    }
  }
  if (has_saved) *values = saved;
  /* 地址分配升级：只迁移旧版记录的DM地址，保留其余已调参数。
   * 新版记录保留用户之后显式DMID设置；迁移仍走原双扇区提交路径。 */
  if (has_saved && loaded_version == 1U) {
    values->value[9] = 3.0f;
    needs_upgrade = 1U;
  }
  /* 空白/无有效记录时沿用固件默认值；首次实际修改才写，避免每次启动擦写。 */
  observed = *values;
  if (!has_saved) saved = *values;
  changed_tick = HAL_GetTick(); state = IDLE;
  return has_saved ? PARAM_STORE_LOADED : PARAM_STORE_DEFAULTS;
}

static ParamStoreResult Fail(void) { state = FAILED; return PARAM_STORE_ERROR; }

ParamStoreResult DebugParamStore_Service(const DebugParamValues *values, uint32_t now)
{
  HAL_StatusTypeDef status;
  uint32_t first, i, address;
  if (state == FAILED) return PARAM_STORE_ERROR;
  if (!DebugParamStore_Valid(values)) return PARAM_STORE_PENDING;
  if (memcmp(values, &observed, sizeof(observed))) { observed = *values; changed_tick = now; }
  if (state == IDLE) {
    if (!needs_upgrade && !memcmp(&observed, &saved, sizeof(saved))) return PARAM_STORE_DEFAULTS;
    if ((uint32_t)(now - changed_tick) < DEBOUNCE_MS) return PARAM_STORE_PENDING;
    pending.magic = MAGIC; pending.version = VERSION; pending.sequence = sequence + 1U;
    pending.size = sizeof(DebugParamValues); pending.params = observed;
    pending.crc = Crc(&pending, 80U); pending.commit = COMMIT;
    first = (active_page / 16U) * 16U;
    for (i = first; i < first + 16U; ++i) if (free_pages & (1UL << i)) break;
    if (i < first + 16U) { target_page = i; state = WRITE_BODY; }
    else {
      target_page = first ^ 16U;
      if (SPIFlash_EraseSector(BASE + target_page * SPI_FLASH_PAGE) != HAL_OK) return Fail();
      free_pages &= ~(0xFFFFUL << target_page);
      operation_tick = now; state = ERASE_WAIT;
    }
    return PARAM_STORE_PENDING;
  }
  address = BASE + target_page * SPI_FLASH_PAGE;
  if (state == ERASE_WAIT || state == BODY_WAIT || state == COMMIT_WAIT) {
    status = SPIFlash_Ready();
    if (status == HAL_ERROR || status == HAL_TIMEOUT) return Fail();
    if (status == HAL_BUSY) {
      if ((uint32_t)(now - operation_tick) > 1000U) return Fail();
      return PARAM_STORE_PENDING;
    }
  }
  if (state == ERASE_WAIT) { check_page = 0U; state = ERASE_CHECK; }
  else if (state == ERASE_CHECK) {
    /* 每周期最多检查一页，避免4KB校验挤占20ms控制周期。 */
    if (SPIFlash_Read(address + check_page * SPI_FLASH_PAGE, page_buffer, SPI_FLASH_PAGE) != HAL_OK || !Blank())
      return Fail();
    if (++check_page == 16U) { free_pages |= 0xFFFFUL << target_page; state = WRITE_BODY; }
  }
  else if (state == WRITE_BODY) {
    free_pages &= ~(1UL << target_page);
    if (SPIFlash_Program(address, &pending, 84U) != HAL_OK) return Fail();
    operation_tick = now; state = BODY_WAIT;
  }
  else if (state == BODY_WAIT) {
    if (SPIFlash_Read(address, &readback, sizeof(readback)) != HAL_OK ||
        memcmp(&readback, &pending, 84U) || readback.commit != 0xFFFFFFFFUL) return Fail();
    if (SPIFlash_Program(address + 84U, &pending.commit, 4U) != HAL_OK) return Fail();
    operation_tick = now; state = COMMIT_WAIT;
  }
  else if (state == COMMIT_WAIT) {
    if (SPIFlash_Read(address, &readback, sizeof(readback)) != HAL_OK ||
        memcmp(&readback, &pending, sizeof(readback)) || !ValidRecord(&readback)) return Fail();
    saved = pending.params; sequence = pending.sequence; active_page = target_page;
    has_saved = 1U; needs_upgrade = 0U; state = IDLE;
    return PARAM_STORE_SAVED;
  }
  return PARAM_STORE_PENDING;
}
