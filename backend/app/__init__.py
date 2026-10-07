import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")  # ponytail: default 24-thread OpenBLAS buffers exceed this host's free commit (4 also crashes here); all BLAS work is tiny, env var overrides
