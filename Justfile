target := `drone print target 2>/dev/null || echo ""`

# Install dependencies
deps:
	type cargo-readme >/dev/null || cargo +stable install cargo-readme
	type drone >/dev/null || cargo install drone
	rustup target add $(drone print target)
	rustup component add clippy
	rustup component add rustfmt

# Reformat the source code
fmt:
	cargo fmt

# Check the source code for mistakes
lint:
	cargo clippy

# Build the documentation
doc:
	cargo doc

# Open the documentation in a browser
doc-open: doc
	cargo doc --open

# Run the tests
test:
	cargo test --features std \
		--target=$(rustc --version --verbose | sed -n '/host/{s/.*: //;p}')

# Update README.md
readme:
	cargo readme -o README.md

# Bump the versions
version-bump version drone-core-version drone-cortexm-version drone-nrf-map-version:
	sed -i "s/\(api\.drone-os\.com\/drone-nrf91-dso\/\)[0-9]\+\(\.[0-9]\+\)\+/\1$(echo {{version}} | sed 's/\(.*\)\.[0-9]\+/\1/')/" \
		Cargo.toml src/lib.rs
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[package\]/version = "{{version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-core\]/version = "{{drone-core-version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-cortexm\]/version = "{{drone-cortexm-version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-nrf-map\]/version = "{{drone-nrf-map-version}}"/;t;x}' \
		Cargo.toml
	sed -i 's/\(drone-nrf91-dso.*\) = "[^"]\+"/\1 = "{{version}}"/' \
		src/lib.rs

# Publish to crates.io
publish:
	cargo publish

# Publish the docs to api.drone-os.com
publish-doc: doc
	dir=$(sed -n 's/.*api\.drone-os\.com\/\(.*\/.*\)\/.*\/"/\1/;T;p' Cargo.toml) \
		&& rm -rf ../drone-api/$dir \
		&& cp -rT target/doc ../drone-api/$dir \
		&& cp -rT target/{{target}}/doc ../drone-api/$dir \
		&& echo '<!DOCTYPE html><meta http-equiv="refresh" content="0; URL=./drone_nrf91_dso">' > ../drone-api/$dir/index.html \
		&& cd ../drone-api && git add $dir && git commit -m "Docs for $dir"
