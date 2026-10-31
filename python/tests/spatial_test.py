import unittest
import numpy as np
import placo
from placo_utils import tf


class TestSpatial(unittest.TestCase):
    def test_exp_log(self):
        """
        exp3/log3 and exp6/log6 are inverse of each other, including near zero and near pi
        """
        rng = np.random.default_rng(0)
        for scale in [1e-9, 1e-3, 0.5, 2.0, 3.1]:
            v = rng.normal(size=6)
            v[3:] *= scale / np.linalg.norm(v[3:])
            self.assertTrue(np.allclose(placo.log6(placo.exp6(v)), v))
            self.assertTrue(np.allclose(placo.log3(placo.exp3(v[3:])), v[3:]))

            R = placo.exp3(v[3:])
            self.assertTrue(np.allclose(R @ R.T, np.eye(3)))

    def test_exp6(self):
        """
        exp6 of a rotation or of a translation
        """
        axis = np.array([1.0, 2.0, 3.0]) / np.linalg.norm([1.0, 2.0, 3.0])
        rotation = placo.exp6(np.array([0.0, 0.0, 0.0, *(0.7 * axis)]))
        self.assertTrue(np.allclose(rotation, tf.rotation_matrix(0.7, axis)))

        translation = placo.exp6(np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(translation, tf.translation_matrix([0.1, 0.2, 0.3])))

    def test_jlog3(self):
        """
        jlog3 is the derivative of log3 (for a rotation applied on the right)
        """
        R = placo.exp3(np.array([0.3, -0.5, 0.8]))
        J = placo.jlog3(R)
        eps = 1e-6
        for k in range(3):
            dw = np.zeros(3)
            dw[k] = eps
            derivative = (placo.log3(R @ placo.exp3(dw)) - placo.log3(R @ placo.exp3(-dw))) / (2 * eps)
            self.assertTrue(np.allclose(J[:, k], derivative, atol=1e-8))

    def test_rpy(self):
        """
        Roll-pitch-yaw conversions
        """
        R = placo.rpy_to_matrix(0.1, -0.4, 2.0)
        expected = tf.rotation_matrix(2.0, [0, 0, 1])[:3, :3] @ tf.rotation_matrix(-0.4, [0, 1, 0])[:3, :3]
        expected = expected @ tf.rotation_matrix(0.1, [1, 0, 0])[:3, :3]
        self.assertTrue(np.allclose(R, expected))
        self.assertTrue(np.allclose(placo.matrix_to_rpy(R), [0.1, -0.4, 2.0]))


if __name__ == "__main__":
    unittest.main()
