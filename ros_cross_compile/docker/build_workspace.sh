#!/bin/bash
set -euxo pipefail

set +ux
# shellcheck source=/dev/null
source "$ros_dir"/setup.bash
set -ux
colcon build --mixin "${TARGET_ARCH}"-docker \
  --build-base build_"${TARGET_ARCH}" \
  --install-base install_"${TARGET_ARCH}"
chown -R "${OWNER_USER}" .
