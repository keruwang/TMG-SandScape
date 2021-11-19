#! /bin/bash
#
# Eigen
#
# uses CMake

# define the version
VER=d20add35bcfc9932671cab9ad786b24fd320a592

# tools for git use
GIT_URL="https://github.com/hmatuschek/eigen3-nnls.git"
GIT_TAG=$VER

FORMULA_TYPES=( "osx" "linux64")

# download the source code and unpack it into LIB_NAME
function download() {
    git clone --depth=1 ${GIT_URL} eigen3-nnls/
    cd eigen3-nnls/
    git checkout -b ${GIT_TAG}
}

# prepare the build environment, executed inside the lib src dir
function prepare() {
  echo "Nothing to prepare."
}

# executed inside the lib src dir
function build() {
  echo "Nothing to build."
}

# executed inside the lib src dir, first arg $1 is the dest libs dir root
function copy() {
    echo `pwd`
    # headers
    if [ -d $1/include ]; then
        rm -rf $1/include
    fi

    mkdir -p $1/include

    cp -r src/ $1/include

}

# executed inside the lib src dir
function clean() {
  echo "Nothing to clean."
}
