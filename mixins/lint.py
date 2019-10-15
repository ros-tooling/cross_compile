#!/usr/bin/env python3

"""Lint the index.yaml file as well as all files ending with .mixin.

Taken from https://github.com/colcon/colcon-mixin-repository.
Modified for the AWS mixins repository.
"""
__author__ = 'Dirk Thomas'
__license__ = 'CC0 1.0 Universal'
__maintainer__ = 'Dirk Thomas'

import logging
import os
import sys


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


try:
    from yamllint.cli import run
except ImportError:
    logger.exception("Failed to import Python package 'yamllint'")
    sys.exit(0)

any_error = False
for name in sorted(os.listdir()):
    if name != 'index.yaml' and not name.endswith('.mixin'):
        continue

    try:
        run([
            '--config-data',
            '{'
            'extends: default, '
            'rules: {'
            'document-start: {present: false}, '
            'empty-lines: {max: 0}, '
            'key-ordering: {}, '
            'line-length: {max: 999}'
            '}'
            '}',
            '--strict',
            name,
        ])
        logger.info('Linting complete.')
    except SystemExit as e:
        any_error |= bool(e.code)
        continue
    assert False, 'run() should always raise SystemExit'

sys.exit(1 if any_error else 0)
