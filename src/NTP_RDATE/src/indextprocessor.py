import gzip
import os

if __name__ == '__main__':
    with open('index.html', 'rb') as orig_file:
        with gzip.open("index.h.gz", 'wb') as zipped_file:
            zipped_file.writelines(orig_file)

    file_size = os.path.getsize('index.h.gz')

    with open('index.h.gz', 'rb') as file_t:
        block = file_t.read()
        st = ""
        for ch in block:
            st += '0x%02X' % ch + ", "
        st = st[:-2]

    with open('index_bytearray.h', 'wt') as text_file:
        text_file.write('#define index_gz_len ' + str(file_size) + '\nconst uint8_t index_gz[] PROGMEM = {\n')

    with open('index_bytearray.h', 'at') as text_file:
        text_file.write(st)

    with open('index_bytearray.h', 'at') as text_file:
        text_file.write('\n};\n')