.PHONY: test test-unit test-integration test-setup test-clean

# Run all tests
test: 
	./reset_realsense.sh
	docker compose -f tests/docker-compose.yml run test pytest -v tests/


# Run only unit tests (no hardware needed)
test-unit: test-setup
	docker compose -f tests/docker-compose.yml run --no-deps test pytest -v tests/unit/


# Run integration tests (requires hardware)
test-integration: 
	./reset_realsense.sh
	docker compose -f tests/docker-compose.yml up --abort-on-container-exit
	# Explicitly specify integration test directory
	docker compose -f tests/docker-compose.yml run test pytest -v tests/integration/

# Build test environment
test-setup:
	docker compose -f tests/docker-compose.yml build

# Clean up test containers and reset realsense
test-clean:
	./reset_realsense.sh
	docker compose -f tests/docker-compose.yml down --rmi all