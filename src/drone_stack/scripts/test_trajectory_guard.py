#!/usr/bin/env python3
import unittest
from types import SimpleNamespace
from geometry_msgs.msg import Point
from trajectory_guard import validate_curve, PackedCells


class CurveChecks(unittest.TestCase):
    def curve(self, points=None):
        if points is None:points=[[0,0,1],[1/3,0,1],[2/3,0,1],[1,0,1]]
        return SimpleNamespace(order=3,knots=[0,0,0,0,1,1,1,1],pos_pts=[Point(*p) for p in points])

    def check(self,b=None,cells=(),lower=(-8,-8,.5),upper=(8,8,2.5),**kwargs):
        return validate_curve(b or self.curve(),set(cells),[0,0,0],.1,lower,upper,**kwargs)

    def test_clear_straight_curve(self):self.assertEqual(self.check(),'')
    def test_collision_between_endpoints(self):self.assertIn('intersects',self.check(cells=[(5,0,10)]))
    def test_voxel_face_touches_lower_cell(self):self.assertIn('intersects',self.check(cells=[(5,-1,9)]))
    def test_upper_height_violation(self):
        b=self.curve([[0,0,1],[0,0,4],[1,0,4],[1,0,1]])
        self.assertIn('volume',self.check(b))
    def test_control_hull_outside_but_actual_curve_inside(self):
        b=self.curve([[0,0,1],[0,3,1],[1,3,1],[1,0,1]])
        self.assertEqual(self.check(b,upper=[8,2.4,2.5]),'')
    def test_hull_obstacle_off_diagonal_curve(self):
        b=self.curve([[0,0,1],[1/3,1/3,1],[2/3,2/3,1],[1,1,1]])
        self.assertEqual(self.check(b,cells=[(5,8,10)]),'')
    def test_unknown_curve_rejected_without_occupied_cells(self):
        self.assertIn('unobserved',self.check(observed_free=set()))
    def test_whole_curve_in_observed_free_cells_allowed(self):
        cells={(x,y,z) for x in range(-1,11) for y in [-1,0] for z in [9,10]}
        self.assertEqual(self.check(observed_free=cells),'')
    def test_one_unknown_cell_between_known_endpoints_rejected(self):
        cells={(x,y,z) for x in range(-1,11) for y in [-1,0] for z in [9,10]}
        cells.remove((5,0,10))
        self.assertIn('unobserved',self.check(observed_free=cells))
    def test_window_allows_known_prefix_without_certifying_unknown_tail(self):
        cells={(x,y,z) for x in range(-1,4) for y in [-1,0] for z in [9,10]}
        self.assertEqual(self.check(observed_free=cells,curve_window=(0,.2)),'')
        self.assertIn('unobserved',self.check(observed_free=cells,curve_window=(.2,.7)))
        self.assertIn('unobserved',self.check(observed_free=cells))
    def test_window_rebases_polynomial_inside_span(self):
        b=self.curve([[0,0,1],[0,0,4],[1,0,4],[1,0,1]])
        self.assertEqual(self.check(b,curve_window=(0,.02)),'')
        self.assertIn('volume',self.check(b,curve_window=(.3,.7)))
    def test_invalid_window_rejected(self):
        self.assertIn('window',self.check(curve_window=(-1,2)))
        self.assertIn('window',self.check(curve_window=(2,1)))
    def test_unsupported_order(self):
        b=self.curve();b.order=2;self.assertIn('unsupported',self.check(b))
    def test_nonfinite_control(self):
        b=self.curve();b.pos_pts[1].z=float('nan');self.assertIn('Malformed',self.check(b))
    def test_decreasing_knots(self):
        b=self.curve();b.knots[4]=-1;self.assertIn('Malformed',self.check(b))
    def test_duration_zero(self):
        b=self.curve();b.knots=[0]*8;self.assertIn('duration',self.check(b))
    def test_node_budget_fail_closed(self):self.assertIn('budget',self.check(node_budget=0))
    def test_wall_budget_fail_closed(self):self.assertIn('budget',self.check(wall_budget=0))
    def test_extreme_controls_fail_without_integer_overflow(self):
        b=self.curve([[-1e100,0,1],[1e100,0,1],[-1e100,0,1],[1e100,0,1]])
        self.assertTrue(self.check(b))


class PackedChecks(unittest.TestCase):
    def test_membership_and_bounds_do_not_alias(self):
        p=PackedCells({(1*3+0)*4+0},[2,3,4])
        self.assertIn((1,0,0),p)
        for q in [(0,3,0),(0,2,4),(2,0,0),(-1,0,0)]: self.assertNotIn(q,p)
    def test_xyz_cloud_decode_matches_floor_cells(self):
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        m=point_cloud2.create_cloud_xyz32(Header(),[[.15,.25,.35],[.15,.25,.35],[-1,0,0],[float('nan'),0,0]])
        p=PackedCells.from_cloud(m,[0,0,0],.1,[10,10,10])
        self.assertEqual(len(p),1);self.assertIn((1,2,3),p)
    def test_empty_cloud(self):
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        p=PackedCells.from_cloud(point_cloud2.create_cloud_xyz32(Header(),[]),[0,0,0],.1,[10,10,10])
        self.assertEqual(len(p),0)
    def test_padded_big_endian_rows(self):
        import struct
        from sensor_msgs.msg import PointCloud2,PointField
        fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')]
        raw=struct.pack('>fff',.15,.25,.35)+b'0000'+struct.pack('>fff',.45,.55,.65)+b'0000'
        m=PointCloud2(height=2,width=1,fields=fields,is_bigendian=True,point_step=12,row_step=16,data=raw)
        p=PackedCells.from_cloud(m,[0,0,0],.1,[10,10,10])
        self.assertEqual(len(p),2);self.assertIn((1,2,3),p);self.assertIn((4,5,6),p)


if __name__=='__main__':unittest.main(verbosity=2)
