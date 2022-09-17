#!/bin/bash
# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`


main()
{
    local start_from=1
    local local_folder=$(pwd)
    local json_file="matrix.json"

    # https://stackoverflow.com/questions/15901239/in-bash-how-do-you-see-if-a-string-is-not-in-an-array
    local skip_docker=("x86" "realsense" "zed")

    # find all docker to build
    local docker=$(find . -type f -name 'Dockerfile.*' | sed 's|.*\.||' | sort -u)
    local n_docker=$(echo "$docker" | wc -w)

    echo " - There are ${yellow}$n_docker docker${reset} in this repository"

    # https://github.blog/changelog/2020-04-15-github-actions-new-workflow-features/
    # https://docs.github.com/en/actions/learn-github-actions/expressions#fromjson
    echo -n "{\"include\":[" > $json_file
    # Worker to build all docker file
    local i=1
    local images=""
    for value in $docker
    do
        local file_name="Dockerfile.$value"
        if [[ " ${skip_docker[*]} " =~ " ${value} " ]] ; then
            echo "${yellow}Skip $file_name${reset}"
            continue
        fi
        images="$images $file_name"
    done

    local matrix="{\"include\":["
    local i=1
    local n_docker=$(echo "$images" | wc -w)

    echo " - ${green}$n_docker docker${reset} to build in the current path"
    for image in $images
    do
        matrix="$matrix{\"project\": \"$(basename $image)\"}"
        echo "- $i Docker: ${green}$file_name${reset}"
        if [ $i != $n_docker ] ; then
            matrix="$matrix,"
        fi
        i=$((i+1))
    done
    matrix="$matrix]}"

    echo $matrix > $json_file

}

main $@
# EOF