const fs = require('fs');
const proc = require('process');

const re1 = /(1\.[0-9]{2})/g;
const re2 = /(1\.[0-9]{2}\.[0-9]{1})/g;

const cargo_toml = 'Cargo.toml';
const files = [
    'README.md',
    '.github/workflows/ci.yml',
    '.github/workflows/clippy.yml',
    '.github/workflows/nightly.yml',
    '.github/workflows/rustfmt.yml'
];

const cargo_toml_src = fs.readFileSync(cargo_toml, 'UTF8').toString();
const cargo_toml_rust_version = cargo_toml_src.split('\n').filter(l => l.startsWith('rust-version'))[0].split('"')[1];

console.log(`Rust version: ${cargo_toml_rust_version}`);

for (const file of files) {
    const file_content = fs.readFileSync(file, 'UTF8').toString();
    const matches1 = file_content.match(re1);
    const matches2 = file_content.match(re2);

    if (matches1) {
        for (const match of matches1) {
            if (match !== cargo_toml_rust_version.substring(0, 4)) {
                console.error(`Found reference to version ${match}, expected ${cargo_toml_rust_version} in file ${file}`);
                proc.exitCode = 1; // Non zero code indicates error
            }
        }
    }

    if (matches2) {
        for (const match of matches2) {
            if (match !== cargo_toml_rust_version) {
                console.error(`Found reference to version ${match}, expected ${cargo_toml_rust_version} in ${file}`);
                proc.exitCode = 1; // Non zero code indicates error
            }
        }
    }

}
