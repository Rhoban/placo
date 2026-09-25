# qpmad (vendored)

[qpmad](https://github.com/asherikov/qpmad) is a header-only QP solver (Goldfarb-Idnani dual active set method) by
Alexander Sherikov, used by PlaCo to solve its QPs (see `src/placo/problem/problem.cpp`).

- Version: 1.5.1 (commit `11602ec9f87a62a97ba8318ebd81da6646f3c631`)
- License: Apache 2.0, see `LICENSE` and `NOTICE` in this directory
- Content: the headers of `include/qpmad/`, unmodified, and `config.h`, generated from
  `config.h.in` with the default options (no tracing, no Householder updates, no pedantic license mode)

To update, copy the headers of a newer qpmad release to `include/qpmad/` and update this file.
