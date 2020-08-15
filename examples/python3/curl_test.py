import cv2
import urllib.request
import numpy as np



############################
# MOBOTIX
url_toplevel = 'http://192.168.0.201'
url = url_toplevel + '/control/faststream.jpg?stream=full&needlength&jpheaderupdate=15&fps=0'
usr = 'admin'
pwd = 'rootroot'
auth = 'basic'

# # AXIS
# url_toplevel = 'http://192.168.0.50'
# url = url_toplevel + '/axis-cgi/mjpg/video.cgi?fps=20'
# usr = 'root'
# pwd = 'root'
# auth = 'digest'


# create a password manager
password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
# Add the username and password.
# If we knew the realm, we could use it instead of None.
password_mgr.add_password(None, url_toplevel, usr, pwd)
if auth == 'basic':
    handler = urllib.request.HTTPBasicAuthHandler(password_mgr)
elif auth == 'digest':
    handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
else:
    raise RuntimeError('Auth not yet supported')
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


print_img_info = True
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
            if print_img_info:
                print('Frame resolution', img.shape)
                print_img_info = False
            if cv2.waitKey(10) == 27:
                break
