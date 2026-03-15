fn main() {
    tonic_build::compile_protos("./proto/drone.proto").unwrap();
}