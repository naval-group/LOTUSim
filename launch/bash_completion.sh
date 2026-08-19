##
## Copyright (c) 2025 Naval Group
##
## This program and the accompanying materials are made available under the
## terms of the Eclipse Public License 2.0 which is available at
## https://www.eclipse.org/legal/epl-2.0.
##
## SPDX-License-Identifier: EPL-2.0
##
lotusim_script_completion() {
    local cur prev opts commands
    COMP_WORDBREAKS=${COMP_WORDBREAKS//:/}
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD - 1]}"

    opts="--ws-path --assets-path --debug --gui --help"
    commands="install clean build clean_build doc run"

    local cmd=""
    for word in "${COMP_WORDS[@]}"; do
        if [[ "$word" == "run" ]]; then
            cmd="$word"
            break
        fi
    done

    case "${prev}" in
    lotusim)
        COMPREPLY=($(compgen -W "${opts} ${commands}" -- ${cur}))
        return 0
        ;;
    --ws-path | --assets-path)
        _filedir
        return 0
        ;;
    esac

    if [[ "$cur" == -* ]]; then
        COMPREPLY=($(compgen -W "${opts}" -- ${cur}))
        return 0
    fi

    if [[ -n "$cmd" ]]; then
        # Mirror how the launcher resolves worlds: the core assets root, plus
        # any --assets-path already typed on this command line.
        local roots=("${LOTUSIM_PATH}/assets") extra_roots i
        for ((i = 0; i < ${#COMP_WORDS[@]} - 1; i++)); do
            if [[ "${COMP_WORDS[i]}" == "--assets-path" ]]; then
                IFS=':' read -ra extra_roots <<< "${COMP_WORDS[i + 1]}"
                roots+=("${extra_roots[@]}")
            fi
        done

        local worlds="" root
        for root in "${roots[@]}"; do
            worlds+=" $(find "${root}/worlds" -type f -name '*.world' 2>/dev/null | sed "s|${root}/worlds/||")"
        done
        COMPREPLY=($(compgen -W "${worlds}" -- ${cur}))
        return 0
    fi
}

xdyn_script_completion() {
    local cur opts
    cur="${COMP_WORDS[COMP_CWORD]}"

    local contains_xdyn=false
    if [[ " ${COMP_WORDS[*]} " == *" xdyn "* ||
        " ${COMP_WORDS[*]} " == *" xdyn-for-ms "* ||
        " ${COMP_WORDS[*]} " == *" xdyn-for-cs "* ]]; then

        opts="--dt --verbose --address --port --help"

        local model_dir=${LOTUSIM_PATH}/assets/models/
        local models=($(ls "$model_dir"))

        if [[ "${cur}" == */* ]]; then
            COMPREPLY=($(compgen -f -- "${cur}"))
        else
            local completions=($(compgen -W "${opts} ${models[*]}" -- "${cur}"))
            if [[ ${#completions[@]} -eq 1 ]]; then
                local found=false
                for model in "${models[@]}"; do
                    if [[ "${completions[0]}" == "$model" ]]; then
                        found=true
                        break
                    fi
                done
                if $found; then
                    COMPREPLY=("$model_dir${completions[0]}/")
                else
                    COMPREPLY=("${completions[@]}")
                fi
            else
                COMPREPLY=("${completions[@]}")
            fi

        fi
        return 0
    fi
}

# Register the completion function for script.sh
complete -F lotusim_script_completion lotusim
complete -F xdyn_script_completion xdyn xdyn-for-me xdyn-for-cs
