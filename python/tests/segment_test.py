import unittest
import placo
import numpy as np

class TestProblem(unittest.TestCase):
    def test_parallelism(self):
        p0 = np.array([0, 0])
        p1 = np.array([1, 1])
        p2 = np.array([1, 0])
        p3 = np.array([2, 1])
        self.assertTrue(placo.Segment(p0, p1).is_parallel(placo.Segment(p2, p3)))
        self.assertFalse(placo.Segment(p0, p1).is_parallel(placo.Segment(p0, p2)))

    def test_point_aligned(self):
        p0 = np.array([0, 0])
        p1 = np.array([1, 1])
        p2 = np.array([2, 2])
        p3 = np.array([2, 1])
        self.assertTrue(placo.Segment(p0, p1).is_point_aligned(p0))
        self.assertTrue(placo.Segment(p0, p1).is_point_aligned(p1))
        self.assertTrue(placo.Segment(p0, p1).is_point_aligned(p2))
        self.assertFalse(placo.Segment(p0, p1).is_point_aligned(p3))

    def test_collinearity(self):
        p0 = np.array([0, 0])
        p1 = np.array([1, 1])
        p2 = np.array([2, 2])
        p3 = np.array([3, 3])
        p4 = np.array([1, 0])
        p5 = np.array([2, 1])
        self.assertTrue(placo.Segment(p0, p1).is_collinear(placo.Segment(p2, p3)))
        self.assertFalse(placo.Segment(p0, p1).is_collinear(placo.Segment(p4, p5)))
        
    def test_point_in_segment(self):
        p0 = np.array([0, 0])
        p1 = np.array([1, 1])
        p2 = np.array([1, 0])
        p3 = np.array([0.5, 0.5])
        p4 = np.array([1.5, 1.5])
        self.assertTrue(placo.Segment(p0, p1).is_point_in_segment(p0))
        self.assertTrue(placo.Segment(p0, p1).is_point_in_segment(p1))
        self.assertFalse(placo.Segment(p0, p1).is_point_in_segment(p2))
        self.assertTrue(placo.Segment(p0, p1).is_point_in_segment(p3))
        self.assertFalse(placo.Segment(p0, p1).is_point_in_segment(p4))
    
    def test_intersection(self):
        p0 = np.array([0, 0])
        p1 = np.array([1, 1])
        p2 = np.array([1, 0])
        p3 = np.array([0, 1])
        p4 = np.array([0.5, 0.5])
        p5 = np.array([2, 2])
        p6 = np.array([3, 3])
        p7 = np.array([-1, 2])
        # Intersect
        self.assertTrue(placo.Segment(p0, p1).intersects(placo.Segment(p2, p3)))
        self.assertFalse(placo.Segment(p5, p6).intersects(placo.Segment(p2, p3)))
        # Lines intersection
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p2, p3))[0], p4[0])
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p2, p3))[1], p4[1])
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p3, p2))[0], p4[0])
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p3, p2))[1], p4[1])
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p2, p3))[0], placo.Segment(p2, p3).lines_intersection(placo.Segment(p0, p1))[0])
        self.assertEqual(placo.Segment(p0, p1).lines_intersection(placo.Segment(p2, p3))[1], placo.Segment(p2, p3).lines_intersection(placo.Segment(p0, p1))[1])
        # Line pass through
        self.assertTrue(placo.Segment(p2, p3).line_pass_through(placo.Segment(p0, p1)))
        self.assertTrue(placo.Segment(p2, p3).line_pass_through(placo.Segment(p5, p6)))
        self.assertFalse(placo.Segment(p5, p6).line_pass_through(placo.Segment(p2, p3)))
        # Half-line pass through
        self.assertTrue(placo.Segment(p0, p1).half_line_pass_through(placo.Segment(p7, p3)))
        self.assertFalse(placo.Segment(p0, p1).half_line_pass_through(placo.Segment(p3, p7)))
        self.assertFalse(placo.Segment(p1, p5).half_line_pass_through(placo.Segment(p7, p3)))

if __name__ == "__main__":
    unittest.main()
