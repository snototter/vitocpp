import cv2
import urllib.request
import numpy as np



############################
#MOBOTIX currently testing
url = 'http://192.168.0.201/control/faststream.jpg?stream=full&needlength&jpheaderupdate=15&fps=0'  # Mobotix

# create a password manager
password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

# Add the username and password.
# If we knew the realm, we could use it instead of None.
top_level_url = "http://192.168.0.201"
password_mgr.add_password(None, top_level_url, 'admin', 'rootroot')
handler = urllib.request.HTTPBasicAuthHandler(password_mgr)
# handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
# create "opener" (OpenerDirector instance)
opener = urllib.request.build_opener(handler)
# use the opener to fetch a URL
opener.open(url)
# Install the opener.
# Now all calls to urllib.request.urlopen use our opener.
urllib.request.install_opener(opener)

def find_(byarr, seq):
#    import pdb; pdb.set_trace()
    for i in range(len(byarr)-1):
        if byarr[i:i+2] == seq:
            return i
    return -1
  
def find_jpg_start(bytes):
    return find_(bytes, b'\xff\xd8')

def find_jpg_end(bytes):
    return find_(bytes, b'\xff\xd9')


with urllib.request.urlopen(url) as stream:
    read_bytes = b''
    while True:
        read_bytes += stream.read(65536)
        a = find_jpg_start(read_bytes)
        b = find_jpg_end(read_bytes)
#        import pdb; pdb.set_trace()
        if a != -1 and b != -1:
            jpg = read_bytes[a:b+2]
            read_bytes = read_bytes[b+2:]
            img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            cv2.imshow('mjpeg over http', img)
            print('Frame resolution', img.shape)
            if cv2.waitKey(10) == 27:
                break




# ############################
# #AXIS STREAMING WORKS
# url = 'http://192.168.0.50/axis-cgi/mjpg/video.cgi?fps=20'  # Axis

# # create a password manager
# password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

# # Add the username and password.
# # If we knew the realm, we could use it instead of None.
# top_level_url = "http://192.168.0.50"
# password_mgr.add_password(None, top_level_url, 'root', 'root')
# #handler = urllib.request.HTTPBasicAuthHandler(password_mgr)#axis uses digest auth
# handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
# # create "opener" (OpenerDirector instance)
# opener = urllib.request.build_opener(handler)
# # use the opener to fetch a URL
# opener.open(url)
# # Install the opener.
# # Now all calls to urllib.request.urlopen use our opener.
# urllib.request.install_opener(opener)

# def find_(byarr, seq):
# #    import pdb; pdb.set_trace()
#     for i in range(len(byarr)-1):
#         if byarr[i:i+2] == seq:
#             print('FOUND CONTROL SEQUENCE!')
#             return i
#     return -1
  
# def find_jpg_start(bytes):
#     return find_(bytes, b'\xff\xd8')

# def find_jpg_end(bytes):
#     return find_(bytes, b'\xff\xd9')


# with urllib.request.urlopen(url) as stream:
#     read_bytes = b''
#     while True:
#         read_bytes += stream.read(4096)
# #        print('80 bytes: ', str(bytes[:80]))
# #        a = bytes.find('\xff\xd8')
# #        b = bytes.find('\xff\xd9')
#         a = find_jpg_start(read_bytes)
#         b = find_jpg_end(read_bytes)
#         print('BYTE LEN', len(read_bytes))
# #        import pdb; pdb.set_trace()
#         if a != -1 and b != -1:
#             jpg = read_bytes[a:b+2]
#             read_bytes = read_bytes[b+2:]
#             img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
#             cv2.imshow('stream?', img)
#             if cv2.waitKey(10) == 27:
#                 break

