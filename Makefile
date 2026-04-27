VERSION := $(shell cat VERSION)

module.tar.gz: run.sh requirements.txt meta.json src/*.py *.so
	tar czf $@ $^

.PHONY: test
test:
	.venv/bin/pip install -q -r requirements-dev.txt
	.venv/bin/pytest

# `make upload` runs the test suite first, then builds the tarball and
# pushes it to the Viam registry at the version recorded in ./VERSION.
# Bump VERSION before invoking this target — the registry rejects
# duplicate version uploads.
.PHONY: upload
upload: test module.tar.gz
	viam module upload --version=$(VERSION) --platform=linux/any module.tar.gz
