from __future__ import print_function

import sys
import re

import glob, os

data_path = sys.argv[0]
descriptor_path = sys.argv[1]

ReservedSbemId_e_Escape = b"\255"
ReservedSbemId_e_Descriptor = 0

#print("data_path:",data_path)
print("descriptor_path:",descriptor_path)

# reads sbem ID upto uint16 from file
def readId(f):
    byte1 = f.read(1)
    id = None
    if not byte1:
        print("EOF found")
    elif byte1 < ReservedSbemId_e_Escape:
        id = int.from_bytes(byte1, byteorder='little')
        #print("one byte id:", id)
    else:
        # read 2 following bytes
        id_bytes = f.read(2)
        id = int.from_bytes(id_bytes, byteorder='little')         
        #print("two byte id:",id)
    return id

# reads sbem length upto uint32 from file
def readLen(f):
    byte1 = f.read(1)
    if byte1 < ReservedSbemId_e_Escape:
        datasize = int.from_bytes(byte1, byteorder='little')
        #print("one byte len:", len)

    else:
        # read 4 following bytes
        id_bytes = f.read(4)
        datasize = int.from_bytes(id_bytes, byteorder='little')         
        #print("4 byte len:",len)
    return datasize

# read sbem chunkheader from file
def readChunkHeader(f):
    id = readId(f)
    if id is None:
        return (None,None)

    datasize = readLen(f)
    ret = (id, datasize)
    print("SBEM chunk header:", ret)
    print(" offset:", f.tell())
    return ret

def readHeader(f):
    # read header
    header_bytes = f.read(8)
    print("SBEM Header: ", header_bytes)

def parseDescriptorChunk(data_bytes):
    print("parseDescriptorChunk data:", chunk_bytes)
    return

def parseDataChunk(data_bytes):
    print("parseDataChunk data:", chunk_bytes)
    return

# read descriptors
with open(descriptor_path, 'rb') as f_desc:

    readHeader(f_desc)

    while True:
        (id, datasize) = readChunkHeader(f_desc)
        if id is None:
#            print("None id:",id)
            break;
        chunk_bytes = f_desc.read(datasize)
        if (len(chunk_bytes) != datasize):
            print("ERROR: too few bytes returned.")
            break

        if id == ReservedSbemId_e_Descriptor:
            parseDescriptorChunk(chunk_bytes)                      
        else:
            print("WARNING: data chunk in descriptor file!")
            parseDataChunk(chunk_bytes)



# read data
with open(data_path, 'rb') as f_data:

    readHeader(f_data)

    while True:
        (id, datasize) = readChunkHeader(f_data)
        if id is None:
#            print("None id:",id)
            break;
        chunk_bytes = f_data.read(datasize)
        if (len(chunk_bytes) != datasize):
            print("ERROR: too few bytes returned.")
            break

        if id == ReservedSbemId_e_Descriptor:
            parseDescriptorChunk(chunk_bytes)                      
        else:
            parseDataChunk(chunk_bytes)