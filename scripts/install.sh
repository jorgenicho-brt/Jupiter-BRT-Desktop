#!/usr/bin/env sh

create_symlinks()
{
  PARENT_DIR=$1
  DEST_DIR=$2
  for FILE in "$PARENT_DIR"/*; do
    if [ -f "$FILE" ]; then
      BASENAME=$(basename ${FILE})
      echo "Creating symlink $DEST_DIR/$BASENAME for file $FILE"
      ln -s $FILE $DEST_DIR/$BASENAME
    fi
  done
}

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

# creating symlinks
BRT_PYTHON_LIBS_PATH="$BRT_APP_PATH/app/brt/usr/lib/python3/brt-packages"
create_symlinks "$BRT_PYTHON_LIBS_PATH/autonomy/adk/idl/common" "$BRT_PYTHON_LIBS_PATH"
create_symlinks "$BRT_PYTHON_LIBS_PATH/autonomy/adk/idl/sensor/camera" "$BRT_PYTHON_LIBS_PATH"

support_script="run_reorganize_packages"
support_script="$(pwd)/../../$support_script"
"$support_script" "$BRT_PYTHON_LIBS_PATH/external" "$BRT_APP_PATH/app/brt/usr/lib/python3/external-packages"
