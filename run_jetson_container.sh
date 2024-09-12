# NOTE: this script is VERY MUCH A WORK IN PROGRESS.
# The ultimate goal is to be able to be able to run a docker container on the jetson 
# that has everything we would need to automatically launch the project with the latest version on github.
# We cannot prebuild an image because that would mean that we would need to build an image every time we have a new version on github.
# Instead what we should do is mount the version inside the docker container with the latest version of the github repo and then build from there

# TODO: pull the most recent version of the code here


# replace the source on the first --mount with the absolute filepath of the sailbotvt repo
sudo docker run -it --privileged \
--mount "source=<ENTER THE ABSOLUTE FILEPATH OF THE REPO HERE>,target=/home/ws/src,type=bind,bind-recursive=enabled" sailbot_container_test:latest \
-v /var/run/docker.sock:/var/run/docker.sock, \
-v /usr/bin/docker:/usr/bin/docker