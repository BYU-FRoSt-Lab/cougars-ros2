
if [ -e "lint_local.sh" ]; then
    cd ..
elif [basename "$PWD" = "cougars-ros2"]; then :
else
    echo "run this script from either the top of the cougars ros2 directory or the .github directory, stopping"
    exit 1
fi
docker run --rm     -e RUN_LOCAL=true     --env-file ".github/super_linter.env"     -v $(pwd):/tmp/lint     ghcr.io/super-linter/super-linter:latest