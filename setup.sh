#!/bin/bash

# Get the current directory
CURRENT_DIR=$(pwd)

# Source the specified environment variables and scripts into ~/.bashrc
echo "export TEMOTO_DIRECTORY=$CURRENT_DIR" >> ~/.bashrc


echo "source $CURRENT_DIR/bash_utils" >> ~/.bashrc

echo 'Setup complete! Resource your .bashrc with "source ~/.bashrc"'