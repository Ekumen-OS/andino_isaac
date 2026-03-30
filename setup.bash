#!/usr/bin/env bash

# Makes andino_isaac available system wide with autocompletion enabled

declare SCRIPT_NAME=$(readlink -f ${BASH_SOURCE[0]})
SCRIPT_PATH=$(dirname ${SCRIPT_NAME})

export WORKSPACE_SOFTWARE_ROOT="${SCRIPT_PATH}"

if [[ "$PYTHONPATH" != *"${WORKSPACE_SOFTWARE_ROOT}/:"* ]]; then
    export PYTHONPATH="${WORKSPACE_SOFTWARE_ROOT}/:${PYTHONPATH}"
fi

if [[ $PATH != *"${WORKSPACE_SOFTWARE_ROOT}/:"* ]]; then
    export PATH="${WORKSPACE_SOFTWARE_ROOT}/:${PATH}"
fi

eval "$(register-python-argcomplete3 andino_isaac)"
