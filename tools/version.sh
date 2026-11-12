#!/usr/bin/env bash
# Compute the rolling project version: YY.MM.NNNN
#   YY.MM  : year.month of the current date (e.g. 26.11)
#   NNNN   : total commit count on HEAD (zero-padded). Monotonically
#            increasing within a month and independent of any human tag
#            maintenance; the YY.MM prefix resets the numbering at each
#            month boundary, so no monthly tag is required to keep the
#            version moving (a missing tag used to freeze every build at
#            YY.MM.00, which made debs of different content indistinguishable
#            to apt).
# The YY.MM tags remain the release convention for sas_py's
# setuptools-git-versioning (the raw tag name becomes the PyPI version);
# they simply no longer feed the deb version.
# Output: 26.11.0018
set -euo pipefail

YYMM="$(date +%y.%m)"

NN=0
if command -v git >/dev/null 2>&1 && git rev-parse --git-dir >/dev/null 2>&1; then
    NN="$(git rev-list HEAD --count 2>/dev/null || echo 0)"
    if [ "${NN}" -eq 0 ]; then
        echo "ERROR: no commits visible (shallow clone?) - refusing to stamp a non-advancing version" >&2
        exit 1
    fi
else
    echo "ERROR: git unavailable - refusing to stamp a non-advancing version" >&2
    exit 1
fi

printf '%s.%04d\n' "${YYMM}" "${NN:-0}"
