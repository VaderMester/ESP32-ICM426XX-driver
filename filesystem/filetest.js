function test() {
    var filesize = get_file_size("cucli.txt");
    print("JS file size is: " + filesize);

    var size = filesize;
    var data = new Buffer(size);
    read_file("cucli.txt", data);

    print(data);

    text = "Valami amerika";
    write_file("cucli2.txt", text);
}

test();
