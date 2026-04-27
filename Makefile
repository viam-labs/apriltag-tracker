module.tar.gz: run.sh requirements.txt meta.json src/*.py *.so
	tar czf $@ $^

.PHONY: test
test:
	.venv/bin/pip install -q -r requirements-dev.txt
	.venv/bin/pytest
