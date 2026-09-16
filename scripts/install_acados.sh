#!/usr/bin/env bash
set -euo pipefail

ACADOS_ROOT="${1:-/opt/acados}"

if [[ ! -d "${ACADOS_ROOT}/.git" ]]; then
  git clone https://github.com/acados/acados.git "${ACADOS_ROOT}" --recursive
fi

cmake -S "${ACADOS_ROOT}" -B "${ACADOS_ROOT}/build" \
  -DACADOS_WITH_QPOASES=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ACADOS_ROOT}"
cmake --build "${ACADOS_ROOT}/build" -j"$(nproc)"
cmake --install "${ACADOS_ROOT}/build"
pip install -e "${ACADOS_ROOT}/interfaces/acados_template"

echo "Add the following to your shell or container environment:"
echo "export ACADOS_SOURCE_DIR=${ACADOS_ROOT}"
echo "export LD_LIBRARY_PATH=${ACADOS_ROOT}/lib:\${LD_LIBRARY_PATH:-}"

