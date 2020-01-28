#!/bin/bash

# ------------------------------
# Install Mac OSX prerequisities
# ------------------------------

# Command Line Tools must be installed, to install run
# xcode-select --install
# Click Install when prompted


brew install cmake wget xz python
# installed:
# cmake		gdbm		gettext		libidn2		libunistring	openssl@1.1	python		readline	sqlite		wget		xz

# Fix gettext link problem
brew unlink gettext && brew link gettext --force

