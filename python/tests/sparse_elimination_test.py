import unittest
import numpy as np
import placo

"""
Exactness of the sparse elimination of equalities (placo.SparseElimination, and Problem.sparse_elimination), on
random equalities A x + b = 0 with various sparse structures.

The elimination expresses the eliminated variables (pivots) as x[pivots] = Z x[free] + x0. It is exact if:
- every point it parametrizes satisfies the equalities (x0 and the columns of Z),
- it parametrizes all the solutions: the number of free variables is n - rank(A), and the solution x* used to generate
  the equalities is recovered from its free variables.
"""


# ---------------------------------------------------------------------------------------------------------------------
# Random structures: each generator returns (n, supports, rows), supports being the (sorted) variables of each equality
# ---------------------------------------------------------------------------------------------------------------------


def independent_blocks(rng):
    """Disjoint blocks of variables, each with its own equalities (loop closures on different legs)"""
    supports, rows, n = [], [], 0
    for _ in range(rng.integers(2, 6)):
        size = int(rng.integers(2, 8))
        block = list(range(n, n + size))
        n += size
        remaining = size - 1
        for _ in range(rng.integers(1, 3)):
            if remaining <= 0:
                break
            r = int(rng.integers(1, remaining + 1))
            cols = sorted(rng.choice(block, size=int(rng.integers(r, size + 1)), replace=False).tolist())
            supports.append(cols)
            rows.append(r)
            remaining -= r
    n += int(rng.integers(0, 4))  # unconstrained variables
    return n, supports, rows


def chain(rng):
    """Equalities coupling a sliding window of consecutive variables (a chain of bodies)"""
    n = int(rng.integers(8, 30))
    width = int(rng.integers(2, 5))
    supports, rows = [], []
    for start in range(0, n - width, int(rng.integers(1, width + 1))):
        supports.append(list(range(start, start + width)))
        rows.append(1)
    return n, supports, rows


def kinematic_tree(rng):
    """
    Degrees of freedom of a random kinematic tree (a floating base, then joints), equalities being loop closures
    between two bodies: their support is the union of the paths from the root to both bodies
    """
    n_bodies = int(rng.integers(4, 14))
    parent = [-1] + [int(rng.integers(0, k)) for k in range(1, n_bodies)]
    dofs, n = [], 0
    for body in range(n_bodies):
        size = 6 if body == 0 else int(rng.integers(1, 3))
        dofs.append(list(range(n, n + size)))
        n += size

    def path(body):
        cols = []
        while body != -1:
            cols += dofs[body]
            body = parent[body]
        return cols

    supports, rows = [], []
    for _ in range(rng.integers(1, 5)):
        a, b = rng.choice(n_bodies, size=2, replace=False)
        cols = sorted(set(path(a)) | set(path(b)))
        supports.append(cols)
        rows.append(int(rng.integers(1, 4)))
    return n, supports, rows


def fixed_base_legs(rng):
    """Legs attached to a fixed base, each closed by a loop (megabot-like): no variable shared between legs"""
    supports, rows, n = [], [], 0
    for _ in range(rng.integers(2, 7)):
        leg = int(rng.integers(3, 8))
        closure = int(rng.integers(1, min(leg, 4)))
        supports.append(list(range(n, n + leg)))
        rows.append(closure)
        n += leg
    return n, supports, rows


def hub(rng):
    """Equalities sharing some hub variables (like a floating base), each with private variables"""
    n_hub = int(rng.integers(1, 7))
    supports, rows, n = [], [], n_hub
    for _ in range(rng.integers(2, 6)):
        private = int(rng.integers(1, 5))
        supports.append(list(range(n_hub)) + list(range(n, n + private)))
        rows.append(int(rng.integers(1, private + 1)))
        n += private
    return n, supports, rows


def random_sparse(rng):
    """Equalities on random subsets of variables (no particular structure)"""
    n = int(rng.integers(6, 40))
    supports, rows = [], []
    for _ in range(rng.integers(1, max(2, n // 3))):
        k = int(rng.integers(1, min(n, 8) + 1))
        supports.append(sorted(rng.choice(n, size=k, replace=False).tolist()))
        rows.append(int(rng.integers(1, k + 1)))
    return n, supports, rows


def square_blocks(rng):
    """Blocks with as many equations as variables, coupled to a separator (the LU path of the elimination)"""
    n_separator = int(rng.integers(0, 4))
    supports, rows, n = [], [], n_separator
    for _ in range(rng.integers(2, 6)):
        size = int(rng.integers(1, 5))
        supports.append(list(range(n_separator)) + list(range(n, n + size)))
        rows.append(size)
        n += size
    return n, supports, rows


def nested(rng):
    """Equalities whose supports are nested or overlapping (generated rows passed along several steps)"""
    n = int(rng.integers(10, 25))
    supports, rows = [], []
    for _ in range(rng.integers(2, 6)):
        start = int(rng.integers(0, n - 3))
        end = int(rng.integers(start + 2, n + 1))
        supports.append(list(range(start, end)))
        rows.append(int(rng.integers(1, 3)))
    return n, supports, rows


GENERATORS = [independent_blocks, chain, kinematic_tree, fixed_base_legs, hub, random_sparse, square_blocks, nested]


def make_equalities(rng, n, supports, rows, row_scales=False, column_scales=None):
    """
    Random equalities A x + b = 0 on the given supports, all satisfied by a random x*. Rows can be scaled (each
    equality by a random power of ten), and columns (variables) as well.
    """
    x_star = rng.normal(size=n)
    As, bs, columns = [], [], []
    for cols, r in zip(supports, rows):
        A = rng.normal(size=(r, len(cols)))
        if row_scales:
            A *= 10.0 ** rng.uniform(-4, 4, size=(r, 1))
        if column_scales is not None:
            A *= column_scales[cols]
        As.append(A)
        bs.append(-A @ x_star[cols])
        columns.append(np.array(cols, dtype=np.int32))
    return x_star, As, bs, columns


def rank(A):
    """Rank of the equalities, rows being normalized (numpy's tolerance depends on the largest singular value)"""
    norms = np.linalg.norm(A, axis=1, keepdims=True)
    return np.linalg.matrix_rank(A / np.where(norms > 0, norms, 1.0))


def dense_matrix(n, As, bs, columns):
    rows = sum(A.shape[0] for A in As)
    A_full, b_full, k = np.zeros((rows, n)), np.zeros(rows), 0
    for A, b, cols in zip(As, bs, columns):
        A_full[k : k + A.shape[0], cols] = A
        b_full[k : k + A.shape[0]] = b
        k += A.shape[0]
    return A_full, b_full


class TestSparseElimination(unittest.TestCase):
    def check_exact(self, n, x_star, As, bs, columns, elimination, msg):
        """Checks that the elimination (which succeeded) parametrizes exactly the solutions of the equalities"""
        A_full, b_full = dense_matrix(n, As, bs, columns)
        pivots, free = np.array(elimination.pivots), np.array(elimination.free)
        Z, x0 = np.array(elimination.Z).reshape(len(pivots), len(free)), np.array(elimination.x0)

        # Pivots and free variables are a partition of the variables
        self.assertEqual(sorted(np.concatenate([pivots, free]).tolist()), list(range(n)), msg=msg)
        self.assertEqual(len(np.unique(pivots)), len(pivots), msg=msg)

        # Z_columns lists the non zero columns of each row of Z
        for row, cols in enumerate(elimination.Z_columns):
            self.assertEqual(list(cols), np.flatnonzero(Z[row]).tolist(), msg=msg)

        def point(z):
            x = np.zeros(n)
            x[free] = z
            x[pivots] = Z @ z + x0
            return x

        # Every parametrized point is a solution (scale-aware tolerance)
        scale = np.abs(A_full).max() * (1 + np.abs(x_star).max())
        for z in [np.zeros(len(free))] + [np.random.default_rng(k).normal(size=len(free)) * 10 for k in range(3)]:
            residual = A_full @ point(z) + b_full
            self.assertLess(np.abs(residual).max(), 1e-9 * scale * (1 + np.abs(z).max(initial=0)), msg=msg)

        # All the solutions are parametrized: dimension of the solutions set, and the generating solution is recovered
        # from its free variables
        self.assertEqual(len(free), n - rank(A_full), msg=msg)
        x = point(x_star[free])
        self.assertLess(np.abs(x - x_star).max(), 1e-8 * (1 + np.abs(x_star).max()), msg=msg)

    def eliminate(self, n, As, bs, columns, **parameters):
        elimination = placo.SparseElimination()
        for key, value in parameters.items():
            setattr(elimination, key, value)
        return elimination, elimination.eliminate(n, As, bs, columns)

    def run_generator(self, generator, seeds, **options):
        used = 0
        for seed in seeds:
            rng = np.random.default_rng(seed)
            n, supports, rows = generator(rng)
            x_star, As, bs, columns = make_equalities(rng, n, supports, rows, **options)
            A_full, _ = dense_matrix(n, As, bs, columns)
            msg = f"{generator.__name__} seed={seed}"

            if rank(A_full) < A_full.shape[0]:
                # Redundant (structurally rank deficient) equalities are refused, as by the QR elimination: either
                # the sparse elimination detects it, or it declines (no structure) and the QR elimination does
                try:
                    _, used_elimination = self.eliminate(n, As, bs, columns)
                    self.assertFalse(used_elimination, msg=msg)
                except RuntimeError:
                    pass
                problem = placo.Problem()
                problem.sparse_elimination = True
                x = problem.add_variable(n)
                for A, b, cols in zip(As, bs, columns):
                    e = placo.Expression()
                    e.A, e.b = A, b
                    problem.add_constraint(e == 0).columns = cols
                with self.assertRaises(RuntimeError, msg=msg):
                    problem.solve()
                continue

            elimination, used_elimination = self.eliminate(n, As, bs, columns)
            if used_elimination:
                used += 1
                self.check_exact(n, x_star, As, bs, columns, elimination, msg)
            else:
                # The elimination is only declined when there is no structure (a single block), or when the
                # parametrization would be poorly conditioned (large Z, typically along chains of substitutions),
                # random blocks being well conditioned (poorly conditioned blocks are tested separately)
                self.assertIn(elimination.status, ["no_structure", "large_Z"], msg=msg)
                if elimination.status == "no_structure":
                    self.assertLessEqual(elimination.steps_count(), 1, msg=msg)
                else:
                    self.assertGreater(elimination.Z_norm, elimination.max_Z_norm, msg=msg)
        return used

    def test_structures(self):
        """
        Exactness on random structures (independent blocks, chains, kinematic trees with loop closures, legs on a fixed
        base, shared hub variables, random sparsity, square blocks, nested supports)
        """
        for generator in GENERATORS:
            used = self.run_generator(generator, range(60))
            # The elimination is actually used for these structures (kinematic trees often have a single block, all the
            # loop closures going through the floating base)
            self.assertGreater(used, 5, msg=generator.__name__)

    def test_scaling(self):
        """Exactness with badly scaled equalities (rows and variables scaled by powers of ten)"""
        for generator in GENERATORS:
            self.run_generator(generator, range(60, 90), row_scales=True)
            for seed in range(90, 110):
                rng = np.random.default_rng(seed)
                n, supports, rows = generator(rng)
                column_scales = 10.0 ** rng.uniform(-3, 3, size=n)
                x_star, As, bs, columns = make_equalities(rng, n, supports, rows, column_scales=column_scales)
                A_full, _ = dense_matrix(n, As, bs, columns)
                if rank(A_full) < A_full.shape[0]:
                    continue
                elimination, used = self.eliminate(n, As, bs, columns)
                if used:
                    self.check_exact(n, x_star, As, bs, columns, elimination, f"{generator.__name__} seed={seed}")

    def test_block_sizes(self):
        """Exactness with any merging of the blocks (max_block_size), from no merging to a single dense block"""
        for max_block_size in [1, 2, 4, 16, 1000]:
            for generator in GENERATORS:
                for seed in range(20):
                    rng = np.random.default_rng(1000 + seed)
                    n, supports, rows = generator(rng)
                    x_star, As, bs, columns = make_equalities(rng, n, supports, rows)
                    A_full, _ = dense_matrix(n, As, bs, columns)
                    if rank(A_full) < A_full.shape[0]:
                        continue
                    elimination, used = self.eliminate(n, As, bs, columns, max_block_size=max_block_size)
                    if used:
                        msg = f"{generator.__name__} seed={seed} max_block_size={max_block_size}"
                        self.check_exact(n, x_star, As, bs, columns, elimination, msg)

    def test_redundant_and_inconsistent(self):
        """Redundant equalities (duplicated or combined rows) and inconsistent ones are refused"""
        for seed in range(30):
            rng = np.random.default_rng(2000 + seed)
            n, supports, rows = fixed_base_legs(rng)
            x_star, As, bs, columns = make_equalities(rng, n, supports, rows)

            # Duplicated equality
            with self.assertRaises(RuntimeError):
                self.eliminate(n, As + [As[0]], bs + [bs[0]], columns + [columns[0]])

            # Linear combination of the rows of an equality, on the same columns
            combination = rng.normal(size=(1, As[0].shape[0]))
            with self.assertRaises(RuntimeError):
                self.eliminate(n, As + [combination @ As[0]], bs + [combination @ bs[0]], columns + [columns[0]])

            # Inconsistent: more independent equations than variables on a leg
            k = len(columns[0])
            A = rng.normal(size=(k + 1, k))
            with self.assertRaises(RuntimeError):
                self.eliminate(n, As + [A], bs + [rng.normal(size=k + 1)], columns + [columns[0]])

    def test_no_structure(self):
        """Without structure (all the equalities in a single block), the elimination is declined"""
        for seed in range(20):
            rng = np.random.default_rng(3000 + seed)
            n = int(rng.integers(3, 12))
            cols = np.arange(n, dtype=np.int32)
            A = rng.normal(size=(int(rng.integers(1, n)), n))
            _, used = self.eliminate(n, [A], [rng.normal(size=A.shape[0])], [cols])
            self.assertFalse(used)

    def test_poorly_conditioned(self):
        """
        Nearly singular blocks: the elimination is either declined (the QR elimination is then used) or exact
        """
        declined = 0
        for seed in range(40):
            rng = np.random.default_rng(4000 + seed)
            n, supports, rows = square_blocks(rng)
            x_star, As, bs, columns = make_equalities(rng, n, supports, rows)
            # A nearly singular square block: two nearly identical rows (or a single tiny row)
            for k, A in enumerate(As):
                if A.shape[0] >= 2:
                    A[1] = A[0] + 1e-10 * rng.normal(size=A.shape[1])
                else:
                    A *= 1e-12
                bs[k] = -A @ x_star[columns[k]]
                break
            try:
                elimination, used = self.eliminate(n, As, bs, columns)
            except RuntimeError:
                # Rank deficiency detected (as by the QR elimination)
                continue
            if used:
                self.check_exact(n, x_star, As, bs, columns, elimination, f"seed={seed}")
            else:
                declined += 1
        self.assertGreater(declined, 0)

    def test_growth(self):
        """
        Along a chain of substitutions, Z can grow exponentially (poorly conditioned parametrization, although exact):
        the elimination is then declined, and the problem solution (using the QR elimination) is exact
        """
        declined = 0
        for seed in range(40):
            rng = np.random.default_rng(9000 + seed)
            n = 30
            # a_k x_k = x_{k+1}, with |a_k| < 1: eliminating x_k from the end of the chain, x_k = x_{k+1} / a_k grows as
            # the product of the 1 / a_k
            supports = [[k, k + 1] for k in range(n - 4)]
            As = [np.array([[rng.uniform(0.25, 0.5) * rng.choice([-1, 1]), -1.0]]) for _ in supports]
            x_star = rng.normal(size=n)
            bs = [-A @ x_star[cols] for A, cols in zip(As, supports)]
            columns = [np.array(cols, dtype=np.int32) for cols in supports]
            elimination, used = self.eliminate(n, As, bs, columns)
            if used:
                self.assertLessEqual(elimination.Z_norm, elimination.max_Z_norm)
                self.check_exact(n, x_star, As, bs, columns, elimination, f"seed={seed}")
            else:
                self.assertEqual(elimination.status, "large_Z")
                declined += 1

                # Without the guard, the parametrization is still exact (but poorly conditioned)
                elimination, used = self.eliminate(n, As, bs, columns, max_Z_norm=np.inf)
                self.assertTrue(used)
                A_full, b_full = dense_matrix(n, As, bs, columns)
                self.assertEqual(len(elimination.free), n - rank(A_full))
        self.assertGreater(declined, 0)

    def test_structure_caching(self):
        """
        The symbolic analysis is cached: successive eliminations with the same structure (new values) or with another
        structure are exact
        """
        elimination = placo.SparseElimination()
        rng = np.random.default_rng(5000)
        structures = [fixed_base_legs(np.random.default_rng(k)) for k in range(3)]
        self.assertEqual(elimination.status, "")
        for step in range(30):
            n, supports, rows = structures[step % 2] if step < 20 else structures[2]
            x_star, As, bs, columns = make_equalities(rng, n, supports, rows)
            self.assertTrue(elimination.eliminate(n, As, bs, columns))
            self.assertEqual(elimination.status, "eliminated")
            self.check_exact(n, x_star, As, bs, columns, elimination, f"step={step}")


class TestSparseEliminationProblem(unittest.TestCase):
    """The sparse elimination in the Problem: same solutions as the QR elimination, no elimination, and a KKT solve"""

    def build(self, mode, n, As, bs, columns, targets, weights, extra=None):
        problem = placo.Problem()
        problem.rewrite_equalities = mode != "none"
        problem.sparse_elimination = mode == "sparse"
        x = problem.add_variable(n)
        for A, b, cols in zip(As, bs, columns):
            e = placo.Expression()
            e.A, e.b = A, b
            problem.add_constraint(e == 0).columns = cols
        # Objective: sum w_i (x_i - t_i)^2
        e = placo.Expression()
        e.A, e.b = np.diag(np.sqrt(weights)), -np.sqrt(weights) * targets
        problem.add_constraint(e == 0).configure("soft", 1.0)
        if extra is not None:
            extra(problem, x)
        problem.solve()
        return problem, x.value.copy()

    def test_kkt(self):
        """Equality constrained least squares: the three modes give the KKT solution"""
        for generator in GENERATORS:
            for seed in range(25):
                rng = np.random.default_rng(6000 + seed)
                n, supports, rows = generator(rng)
                x_star, As, bs, columns = make_equalities(rng, n, supports, rows)
                A_full, b_full = dense_matrix(n, As, bs, columns)
                if rank(A_full) < A_full.shape[0]:
                    continue
                targets, weights = rng.normal(size=n) * 2, 10.0 ** rng.uniform(-2, 2, size=n)

                # KKT: [H A^T; A 0] [x; l] = [-g; -b], with H = 2 W (+ the regularization of the problem)
                m = A_full.shape[0]
                H = np.diag(2 * weights) + 2 * 1e-8 * np.eye(n)
                K = np.block([[H, A_full.T], [A_full, np.zeros((m, m))]])
                x_kkt = np.linalg.solve(K, np.concatenate([2 * weights * targets, -b_full]))[:n]

                msg = f"{generator.__name__} seed={seed}"
                for mode in ["sparse", "qr", "none"]:
                    problem, x = self.build(mode, n, As, bs, columns, targets, weights)
                    self.assertLess(np.abs(x - x_kkt).max(), 1e-6 * (1 + np.abs(x_kkt).max()), msg=f"{msg} {mode}")
                    self.assertLess(np.abs(A_full @ x + b_full).max(), 1e-8 * (1 + np.abs(b_full).max()), msg=msg)

    def test_with_inequalities_and_bounds(self):
        """Bounds (on free and eliminated variables), hard and soft inequalities: same solution in the three modes"""
        for generator in GENERATORS:
            for seed in range(15):
                rng = np.random.default_rng(7000 + seed)
                n, supports, rows = generator(rng)
                x_star, As, bs, columns = make_equalities(rng, n, supports, rows)
                A_full, _ = dense_matrix(n, As, bs, columns)
                if rank(A_full) < A_full.shape[0]:
                    continue
                targets, weights = rng.normal(size=n) * 3, 10.0 ** rng.uniform(-1, 1, size=n)
                bounded = rng.choice(n, size=min(n, 4), replace=False)
                margin = rng.uniform(0.1, 1.0, size=len(bounded))
                G = rng.normal(size=(2, n))
                h = -G @ x_star + 0.5  # satisfied by x*

                def extra(problem, x):
                    for i, k in enumerate(bounded):
                        # x* is within its bounds, so that the problem stays feasible
                        problem.add_bounds(x, int(k), np.array([x_star[k] - margin[i]]), np.array([x_star[k] + margin[i]]))
                    e = placo.Expression()
                    e.A, e.b = G, h
                    problem.add_constraint(e >= 0)
                    e2 = placo.Expression()
                    e2.A, e2.b = G[:1], h[:1] - 1.0
                    problem.add_constraint(e2 >= 0).configure("soft", 5.0)

                msg = f"{generator.__name__} seed={seed}"
                solutions = [self.build(mode, n, As, bs, columns, targets, weights, extra)[1] for mode in ["qr", "sparse", "none"]]
                for solution in solutions[1:]:
                    self.assertLess(np.abs(solution - solutions[0]).max(), 1e-6 * (1 + np.abs(solutions[0]).max()), msg=msg)

    def test_sparse_used_and_fallback(self):
        """The problem reports whether the sparse elimination was used, and falls back to QR otherwise"""
        rng = np.random.default_rng(8000)
        n, supports, rows = fixed_base_legs(rng)
        _, As, bs, columns = make_equalities(rng, n, supports, rows)
        targets, weights = rng.normal(size=n), np.ones(n)
        problem, _ = self.build("sparse", n, As, bs, columns, targets, weights)
        self.assertTrue(problem.sparse_elimination_used)

        # A single dense block: fallback to QR (same solution)
        A = rng.normal(size=(2, n))
        dense = ([A], [rng.normal(size=2)], [np.arange(n, dtype=np.int32)])
        problem, x_sparse = self.build("sparse", n, *dense, targets, weights)
        self.assertFalse(problem.sparse_elimination_used)
        _, x_qr = self.build("qr", n, *dense, targets, weights)
        self.assertLess(np.abs(x_sparse - x_qr).max(), 1e-9)

    def test_invalid_columns(self):
        """Compact constraints columns must be strictly increasing and within the variables"""
        for columns in [[0, 0, 1], [2, 1, 0], [0, 1, 5], [-1, 0, 1]]:
            problem = placo.Problem()
            problem.add_variable(4)
            e = placo.Expression()
            e.A, e.b = np.ones((1, 3)), np.zeros(1)
            problem.add_constraint(e == 0).columns = np.array(columns, dtype=np.int32)
            with self.assertRaises(RuntimeError, msg=str(columns)):
                problem.solve()


if __name__ == "__main__":
    unittest.main()
