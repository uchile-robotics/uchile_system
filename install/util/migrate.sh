#!/bin/bash

function update_remote ()
{
    local remote path branches branch local_branch cur_hash github_hash old_path
    remote="$1"
    path="$2"

    cd "${HOME}/bender_ws/${path}"
    
    # fetch
    echo "- fetching from origin ..."
    git fetch origin
    echo "- fetching from ${remote} ..."
    git fetch "$remote"

    # remove outdated remote branches
    echo "- pruning origin ..."
    git remote prune origin
    echo "- pruning ${remote} ..."
    git remote prune "${remote}"

    # for each branch in origin/
    branches=()
    eval "$(git for-each-ref --shell --format='branches+=(%(refname))' refs/remotes/origin)"
    for branch in "${branches[@]}"; do
        
        local_branch="${branch/refs\/remotes\/origin\/}"
        if [ "${local_branch}" = "HEAD" ]; then
            continue
        fi
        # avoid feat-uchile
        if [ "${local_branch}" = "feat-uchile" ]; then
            continue
        fi
        
        echo "-----------------------------------"
        echo "RAMA: ${local_branch}"
        if [ "${local_branch}" = "develop" ]; then
            git checkout feat-uchile > /dev/null
            git merge origin/feat-uchile
            git merge "${remote}"/feat-uchile
            git merge origin/develop
            local_branch="feat-uchile"
        else
            git checkout "${local_branch}" > /dev/null
            git merge origin/"${local_branch}"
        fi
        cur_hash=$(git rev-parse --verify "HEAD")
        origin_hash=$(git rev-parse --verify "origin/${local_branch}")
        github_hash=$(git rev-parse --verify "${remote}/${local_branch}")
        if [ "${cur_hash}" = "${origin_hash}" ]; then
            echo " - branch local (${local_branch}) ya sincronizada con origin (origin/${local_branch})"
        else
            git push origin "${local_branch}"
            echo "git push origin ${local_branch}"
        fi
        if [ "${cur_hash}" = "${github_hash}" ]; then
            echo " - branch local (${local_branch}) ya sincronizada con github (${remote}/${local_branch})"
        else
            git push "${remote}" "${local_branch}"
            echo "git push ${remote} ${local_branch}"
        fi
    done
    git checkout develop    
}

# update_remote "github-common" "base_ws/src"
# update_remote "github-bender" "base_ws/src"
# update_remote "github-tools"  "base_ws/src"

# update_remote "github" "soft_ws/src/uchile_navigation"
# update_remote "github" "soft_ws/src/uchile_perception"
# update_remote "github" "soft_ws/src/uchile_manipulation"
# update_remote "github" "soft_ws/src/uchile_hri"

# update_remote "github" "high_ws/src/uchile_high"

# cdb uchile_navigation
# git fetch
# git checkout feat-uchile
# git merge origin/feat-uchile
# git merge origin/develop
# git push
# git checkout develop
# git merge origin/develop
# git merge origin/feat-uchile
# git push


# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat-calibration-arm
# git merge origin/feat-calibration-arm
# git push github-common
# git push github-tools
# git push github-bender

# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat-depfix
# git merge origin/feat-depfix
# git push github-common
# git push github-tools
# git push github-bender

# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat-hwcheck
# git merge origin/feat-hwcheck
# git push github-common
# git push github-tools
# git push github-bender

# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat-new-emotion
# git merge origin/feat-new-emotion
# git push github-common
# git push github-tools
# git push github-bender

# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat/headjoy
# git merge origin/feat/headjoy
# git push github-common
# git push github-tools
# git push github-bender

# cd "${HOME}/bender_ws/base_ws/src"
# git checkout feat-navigation
# git merge origin/feat-navigation
# git push github-common
# git push github-tools
# git push github-bender

