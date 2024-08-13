#!/usr/bin/env sh
BRT_APP_PATH="$HOME"
if [ "$#" -ge 1 ]; then
    BRT_APP_PATH=$1
    echo "SET BRT_APP_PATH=$1"
else
    echo "BRT_APP_PATH not set, using default $BRT_APP_PATH"
fi

CURRENT_DIR="$(dirname "$0")"
DESKTOP_TAR_FILE="${CURRENT_DIR}/desktop_app.tar"
if  [ -f ${DESKTOP_TAR_FILE} ]; then
  echo "Found tar file: ${DESKTOP_TAR_FILE}"
else
  echo "Invalid path to tar file ${DESKTOP_TAR_FILE}"
  exit 1
fi

echo "Installing in directory ${BRT_APP_PATH} ..."
tar -xvpf $DESKTOP_TAR_FILE -C $BRT_APP_PATH
