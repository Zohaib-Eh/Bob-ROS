import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "4")  # ponytail: default 24-thread OpenBLAS buffers exceed this host's free commit; env var overrides
