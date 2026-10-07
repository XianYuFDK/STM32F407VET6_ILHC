import math
import unittest
import random
import threading
from unittest.mock import patch
from Docs.audit_route_optimality_20261005 import dijkstra,skeleton_cost
from coordinate_search import RankedRoutes


class GraphOracleTests(unittest.TestCase):
    def test_exact_diagonal_and_turn_weighted_route_cost(self):
        s,a,b,g=(0,0),(100,0),(50,50),(100,100)
        graph={s:[a,b],a:[g],b:[g],g:[]}
        cost,path=dijkstra(graph,s,g)
        self.assertEqual(path,[s,b,g])
        self.assertAlmostEqual(cost,math.sqrt(20000))
        self.assertAlmostEqual(cost,skeleton_cost(path))

    def test_incoming_direction_state_is_required_for_optimal_bend_cost(self):
        s,a,b,m,g=(0,0),(100,0),(0,100),(100,100),(200,100)
        graph={s:[a,b],a:[m],b:[m],m:[g],g:[]}
        cost,path=dijkstra(graph,s,g)
        self.assertEqual(cost,420)
        self.assertEqual(path,[s,b,m,g])

    def test_disconnected_graph_reports_no_optimum(self):
        self.assertEqual(dijkstra({(0,0):[(100,0)],(100,0):[(0,0)]},(0,0),(200,0)),(None,None))


class OrderedSkeletonTests(unittest.TestCase):
    def test_all_costs_match_independent_exhaustive_simple_path_oracle(self):
        # Independent DFS, deliberately no production heuristic/dominance.
        points=[(0,0),(100,0),(100,100),(0,100),(200,100),(200,0)]
        rng=random.Random(71)
        for _ in range(30):
            graph={a:[b for b in points if a!=b and rng.random()<.45] for a in points}
            expected={}
            def visit(path):
                if path[-1]==points[-1]:
                    compressed=[]
                    for p in path:
                        if len(compressed)>1:
                            a,b=compressed[-2:];u=(b[0]-a[0],b[1]-a[1]);v=(p[0]-b[0],p[1]-b[1])
                            if abs(u[0]*v[1]-u[1]*v[0])<1e-7 and u[0]*v[0]+u[1]*v[1]>0 and p in graph[a]:
                                compressed.pop()
                        compressed.append(p)
                    expected[tuple(compressed)]=skeleton_cost(compressed)
                    return
                for target in graph[path[-1]]:
                    if target in path:continue
                    if len(path)>1:
                        a,b=path[-2:];u=(b[0]-a[0],b[1]-a[1]);v=(target[0]-b[0],target[1]-b[1])
                        if abs(u[0]*v[1]-u[1]*v[0])<1e-7 and u[0]*v[0]+u[1]*v[1]<0:continue
                    visit(path+[target])
            visit([points[0]])
            search=RankedRoutes(graph,points[0],points[-1]);actual={};costs=[]
            while True:
                result=search.next()
                if result is None:break
                cost,path=result;costs.append(cost);actual[tuple(path)]=cost
            self.assertEqual(set(expected),set(actual))
            for path,cost in actual.items():
                self.assertAlmostEqual(cost,skeleton_cost(path))
            self.assertEqual(costs,sorted(costs))
            if expected:self.assertAlmostEqual(costs[0],min(expected.values()))

    def test_turn_state_lower_bound_matches_independent_dijkstra(self):
        s,a,b,m,g=(0,0),(100,0),(0,100),(100,100),(200,100)
        graph={s:[a,b],a:[m],b:[m],m:[g],g:[]}
        search=RankedRoutes(graph,s,g)
        self.assertEqual(search.initial_lower_bound,dijkstra(graph,s,g)[0])
        self.assertEqual(search.next(),(420,[s,b,m,g]))

    def test_missing_direct_edge_is_not_invented_by_collinear_compression(self):
        s,a,g=(0,0),(100,0),(200,0)
        search=RankedRoutes({s:[a],a:[g],g:[]},s,g)
        self.assertEqual(search.next(),(200,[s,a,g]))

    def test_upper_bound_prunes_only_provably_more_expensive_routes(self):
        s,a,b,g=(0,0),(100,0),(50,50),(100,100)
        search=RankedRoutes({s:[a,b],a:[g],b:[g],g:[]},s,g)
        self.assertAlmostEqual(search.next(200)[0],math.sqrt(20000))
        self.assertIsNone(search.next(200))
        self.assertFalse(search.limit_hit)

    def test_budget_does_not_claim_search_exhausted_and_cancel_is_observed(self):
        graph={(0,0):[(100,0)],(100,0):[]}
        search=RankedRoutes(graph,(0,0),(100,0),max_expanded=0)
        self.assertIsNone(search.next());self.assertTrue(search.limit_hit)
        self.assertEqual(search.remaining_lower_bound,100)
        with self.assertRaisesRegex(ValueError,'取消'):
            RankedRoutes(graph,(0,0),(100,0),cancelled=lambda:True).next()

    def test_controller_selection_beats_old_first_safe_and_compares_all_settings(self):
        import competition_simulation as competition
        import coordinate_navigation as coordinate
        from Docs.audit_route_optimality_20261005 import better_replays
        data=competition.load_profile();scene=competition.collision_scene(data)
        result=coordinate.plan_route((2100,2100),tuple(data['competition']['stations']['qr']),scene,
                                      data['competition']['lane_nodes'],180,pivot_turns=False,approach_turns=False)
        baseline=result['optimality']['baseline']
        self.assertEqual(result['optimality']['proven'],all(p['complete'] for p in result['search']['phases']))
        self.assertFalse(result['optimality']['global_time_proven'])
        self.assertLessEqual(result['execution_cost']['score_s'],baseline['execution_cost']['score_s'])
        self.assertEqual(result['optimality']['policy'],'VERIFIED_TIME_AND_YAW_COST')
        self.assertLessEqual(result['search']['candidates'],coordinate.MAX_OPTIMIZATION_SKELETONS)

    def test_rough_storage_uses_tail_and_outer_wheel_turn_with_terminal_work_heading(self):
        import competition_simulation as competition
        import coordinate_navigation as coordinate
        data=competition.load_profile();scene=competition.collision_scene(data)
        result=coordinate.plan_route((1200,400),(400,1200),scene,data['competition']['lane_nodes'],0,goal_yaw=270)
        first=result['trajectory'][0]
        moved=next(p for p in result['trajectory'] if math.dist((p['x_mm'],p['y_mm']),(first['x_mm'],first['y_mm']))>10)
        # 返回轨迹是场地坐标；LAYOUT沿车尾-x对应场地+Y。
        self.assertGreater(moved['y_mm'],first['y_mm'])
        self.assertLess(abs(coordinate.wrap(moved['field_yaw_deg']-first['field_yaw_deg'])),1)
        self.assertNotIn((1200,1200),result['skeleton_points'])
        self.assertLess(result['execution_cost']['yaw_total_deg'],100)
        self.assertTrue(result.get('pivots'));self.assertFalse(result['optimality']['proven'])
        samples,elapsed=coordinate.replay(result['waypoint_program'],scene)
        self.assertAlmostEqual(elapsed,result['predicted_tracking_s'])
        goal=coordinate.CoordinateTracker(result['waypoint_program']).final_reference()
        self.assertLess(abs(coordinate.wrap(samples[-1]['field_yaw_deg']-goal['field_yaw_deg'])),1)

    def test_goal_aware_plan_can_disable_wheel_extension(self):
        import coordinate_navigation as coordinate
        from navigation_planner import CollisionScene
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        result=coordinate.plan_route((500,500),(1500,1500),scene,[(500,1500),(1500,500)],0,goal_yaw=270,pivot_turns=False)
        self.assertFalse(any(w.get('motion')=='WHEEL_PIVOT' for w in result['waypoint_program']['waypoints']))

    def test_budget_exhaustion_retains_safe_seed_without_false_optimality_claim(self):
        import coordinate_navigation as coordinate
        from navigation_planner import CollisionScene
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        def limited(*args,**kwargs):return RankedRoutes(*args,**kwargs,max_expanded=0)
        with patch('coordinate_search.RankedRoutes',side_effect=limited):
            result=coordinate.plan_route((500,500),(1500,500),scene,[],0)
        self.assertTrue(result['execution_safe'])
        self.assertFalse(result['optimality']['proven'])
        self.assertTrue(any(p['limit_hit'] for p in result['search']['phases']))

    def test_cancel_interrupts_actual_optimizer_without_returning_incumbent(self):
        import coordinate_navigation as coordinate
        import competition_simulation as competition
        data=competition.load_profile();scene=competition.collision_scene(data,sim_obstacles=((1200,1200),(297,294)))
        event=threading.Event();entered=threading.Event();results=[];errors=[]
        real=coordinate._cached_replay
        def observe(*args):entered.set();return real(*args)
        def run():
            try:results.append(coordinate.plan_route(tuple(data['competition']['stations']['rough']),
                tuple(data['competition']['stations']['storage']),scene,data['competition']['lane_nodes'],
                -180,cancelled=event.is_set))
            except ValueError as exc:errors.append(exc)
        with patch.object(coordinate,'_cached_replay',side_effect=observe):
            worker=threading.Thread(target=run);worker.start()
            self.assertTrue(entered.wait(5));event.set();worker.join(3)
        self.assertFalse(worker.is_alive());self.assertEqual(results,[])
        self.assertEqual(len(errors),1);self.assertIn('取消',str(errors[0]))

    def test_replay_cache_respects_parameter_constraint_changes_and_cancellation(self):
        import coordinate_navigation as coordinate
        from navigation_planner import CollisionScene
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None);cache={}
        start=[(500,500),(500,1500)]
        safe=coordinate.build_program(start,90)
        old=coordinate._cached_replay(safe,scene,lambda:False,{},cache)
        slow=coordinate.build_program(start,90,control={'xyvmax':200})
        new=coordinate._cached_replay(slow,scene,lambda:False,{},cache)
        self.assertGreater(new[1],old[1])
        lateral=coordinate.build_program(start,0,mode='FIXED',constraints={'strafe_run_limit_mm':100})
        with self.assertRaisesRegex(ValueError,'横移'):
            coordinate._cached_replay(lateral,scene,lambda:False,{},cache)
        with self.assertRaisesRegex(ValueError,'取消'):
            coordinate._cached_replay(safe,scene,lambda:True,{},cache)

    def test_dense_replay_cache_is_bounded_by_samples_not_only_route_count(self):
        import coordinate_navigation as coordinate
        cache={};samples=[{'x_mm':0}]*15001
        with patch.object(coordinate,'replay',return_value=(samples,1)) as replay:
            for i in range(4):
                coordinate._cached_replay({'test_control':i},None,lambda:False,{},cache)
                self.assertLessEqual(sum(len(v[0]) for v in cache.values() if isinstance(v,tuple)),30000)
            coordinate._cached_replay({'test_control':0},None,lambda:False,{},cache)
            self.assertEqual(replay.call_count,5)

    def test_yaw_scoring_includes_first_integration_and_final_residual(self):
        import coordinate_navigation as coordinate
        program=coordinate.build_program([(500,500),(500,1500)],0,goal_yaw=270)
        cost=coordinate.execution_cost(program,[{'field_yaw_deg':-50}],1)
        self.assertEqual(cost['yaw_total_deg'],40)
        self.assertEqual(cost['yaw_residual_deg'],50)
        self.assertAlmostEqual(cost['score_s'],2.35)

    def test_deadline_retains_verified_incumbent_but_never_claims_optimum(self):
        import coordinate_navigation as coordinate
        from navigation_planner import CollisionScene
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        with patch.object(coordinate,'_coordinate_graph',side_effect=coordinate.OptimizationDeadline):
            result=coordinate.plan_route((500,500),(1500,500),scene,[],0)
        self.assertTrue(result['execution_safe'])
        self.assertFalse(result['optimality']['proven'])
        self.assertTrue(result['search']['time_limit_hit'])
        samples,elapsed=coordinate.replay(result['waypoint_program'],scene)
        self.assertEqual(elapsed,result['predicted_tracking_s'])

    def test_deadline_without_incumbent_and_explicit_cancel_refuse_a_plan(self):
        import coordinate_navigation as coordinate
        from navigation_planner import CollisionScene
        scene=CollisionScene([],[],(0,0,3000,3000),10,(280,260,0),None)
        with self.assertRaisesRegex(ValueError,'时间预算'):
            coordinate.plan_route((500,500),(1500,500),scene,[],0,deadline=0)
        with self.assertRaisesRegex(ValueError,'取消'):
            coordinate.plan_route((500,500),(1500,500),scene,[],0,deadline=0,cancelled=lambda:True)

    def test_core_time_limit_is_distinct_from_user_cancel(self):
        import core
        args=dict(footprint=(280,260,0),bounds=(0,0,3000,3000),rects=[],circles=[],time_limit_s=0)
        result=core.plan_coordinate_path((500,500),(1500,500),**args)
        self.assertFalse(result['ok']);self.assertEqual(result['code'],'TIME_LIMIT')
        event=threading.Event();event.set()
        result=core.plan_coordinate_path((500,500),(1500,500),cancel=event,**args)
        self.assertFalse(result['ok']);self.assertEqual(result['code'],'CANCELLED')


if __name__=='__main__':unittest.main()
