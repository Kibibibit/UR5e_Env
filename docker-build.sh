#!/bin/bash



./.helper-scripts/get-user-confirmation.sh $@ \
    -m "Are you sure? This will delete your exising container and image, and take potentially a long time  (About 5-10 minutes) to rebuild!"

if [ $? -eq 0 ]
then

    # Delete the previous container
    ./docker-delete.sh -y

    # Set up the required xauth files for the display bridge
    XSOCK=/tmp/.X11-unix
    XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    # Assuming that passed, we build the docker
    if [ $? -eq 0 ]
    then
        USER_UID="$(id -u)" \
            USER_GID="$(id -g)" \
            USERNAME="rosuser" \
            docker-compose build
    else
        # Otherwise print an error.
        echo -e "\033[0;31mERROR: Failed to set up XAUTH files. This probably means the container was shut down unexpectedly. To resolve, run:\033[0m\n\
sudo rm -rf $XAUTH\n\
\033[0;31mand try again.\033[0m"
    fi


    
        
    
fi



