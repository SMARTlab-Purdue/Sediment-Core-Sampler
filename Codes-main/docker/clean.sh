#/bin/bash -x

docker rm -f $(docker ps --filter "name=wabash*" -q)
