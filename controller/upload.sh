#!/bin/zsh

# Not really sure why this is necessary, but it is
export DYLD_LIBRARY_PATH=/usr/local/lib:$DYLD_LIBRARY_PATH
pro32 -p -v build/controller.hex