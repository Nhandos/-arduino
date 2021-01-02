TARGET=DetectHeading.ino
FULLY_QUALIFIED_BOARD_NAME=arduino:avr:nano
LIBRARIES_PATH=./libraries/

arduino-cli compile --fqbn $FULLY_QUALIFIED_BOARD_NAME \
                    --libraries $LIBRARIES_PATH \
                    $TARGET

