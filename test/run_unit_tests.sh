# TODO: Remove this once we get our own dockerhub repo
$(aws ecr get-login --no-include-email --region us-east-2)
py.test test_sysroot_creator.py