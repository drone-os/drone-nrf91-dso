cortex_m_core := 'cortex_m33f_r0p2'
nrf_mcu := 'nrf9160'
export DRONE_RUSTFLAGS := '--cfg cortex_m_core="' + cortex_m_core + '" ' + '--cfg nrf_mcu="' + nrf_mcu + '"'
target := 'thumbv8m.main-none-eabihf'

# Install dependencies
deps:
	rustup target add {{target}}
	rustup component add clippy
	rustup component add rustfmt
	type cargo-readme >/dev/null || cargo +stable install cargo-readme

# Reformat the source code
fmt:
	cargo fmt

# Check for mistakes
lint:
	drone env {{target}} -- cargo clippy

# Generate the docs
doc:
	drone env {{target}} -- cargo doc

# Open the docs in a browser
doc-open: doc
	drone env {{target}} -- cargo doc --open

# Run the tests
test:
	drone env -- cargo test --features "std"

# Update README.md
readme:
	cargo readme -o README.md

# Bump crate versions
version-bump version drone-core-version drone-cortex-m-version drone-nrf-map-version:
	sed -i "s/\(api\.drone-os\.com\/drone-nrf91-uart-log\/\)[0-9]\+\(\.[0-9]\+\)\+/\1$(echo {{version}} | sed 's/\(.*\)\.[0-9]\+/\1/')/" \
		Cargo.toml src/lib.rs
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[package\]/version = "{{version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-core\]/version = "{{drone-core-version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-cortex-m\]/version = "{{drone-cortex-m-version}}"/;t;x}' \
		Cargo.toml
	sed -i '/\[.*\]/h;/version = ".*"/{x;s/\[.*drone-nrf-map\]/version = "{{drone-nrf-map-version}}"/;t;x}' \
		Cargo.toml
	sed -i 's/\(drone-nrf91-uart-log.*\)version = "[^"]\+"/\1version = "{{version}}"/' \
		src/lib.rs

# Publish to crates.io
publish:
	drone env {{target}} -- cargo publish

# Publish the docs to api.drone-os.com
publish-doc: doc
	dir=$(sed -n 's/.*api\.drone-os\.com\/\(.*\/.*\)\/.*\/"/\1/;T;p' Cargo.toml) \
		&& rm -rf ../drone-api/$dir \
		&& cp -rT target/doc ../drone-api/$dir \
		&& cp -rT target/{{target}}/doc ../drone-api/$dir \
		&& echo '<!DOCTYPE html><meta http-equiv="refresh" content="0; URL=./drone_nrf91_uart_log">' > ../drone-api/$dir/index.html \
		&& cd ../drone-api && git add $dir && git commit -m "Docs for $dir"
