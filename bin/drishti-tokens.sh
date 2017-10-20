#!/bin/bash

TOKEN=$1

# Github release:
#
# https://github.com/forexample/github-binary-release

function github_release
{
    travis logout && travis --login --org --github-token=${TOKEN} && travis encrypt -r elucideye/drishti ${TOKEN}
}

echo "drishti:deploy:api_key:"
github_release

# Hunter cache:
# 
# https://github.com/ruslo/hunter/blob/master/docs/faq/why-binaries-from-server-not-used.rst

function github_upload
{
    travis logout && travis --login --org --github-token=${TOKEN} && travis encrypt GITHUB_USER_PASSWORD=${TOKEN}
}

echo "drishti-upload:env.global:"
(cd drishti-upload && github_upload)

