#include "curl_file_handling.h"

#include <stdio.h>
#include <string.h>
#ifndef WIN32
#include <sys/time.h>
#endif
#include <stdlib.h>
#include <errno.h>


namespace vcp
{
namespace best
{
namespace curl
{
/* curl calls this routine to get more data */
static size_t write_callback(char *buffer,
                             size_t size,
                             size_t nitems,
                             void *userp)
{
  char *newbuff;
  size_t rembuff;

  URL_FILE *url = (URL_FILE *)userp;
  size *= nitems;

  rembuff=url->buffer_len - url->buffer_pos; /* remaining space in buffer */

  if(size > rembuff) {
    /* not enough space in buffer */
    newbuff=(char*)realloc(url->buffer,url->buffer_len + (size - rembuff));
    if(newbuff==NULL) {
      fprintf(stderr,"callback buffer grow failed\n");
      size=rembuff;
    }
    else {
      /* realloc suceeded increase buffer size*/
      url->buffer_len+=size - rembuff;
      url->buffer=newbuff;
    }
  }

  memcpy(&url->buffer[url->buffer_pos], buffer, size);
  url->buffer_pos += size;

  return size;
}

/* use to attempt to fill the read buffer up to requested number of bytes */
static int fill_buffer(CURLM *multi_handle, URL_FILE *file, size_t want)
{
  fd_set fdread;
  fd_set fdwrite;
  fd_set fdexcep;
  struct timeval timeout;
  int rc;

  /* only attempt to fill buffer if transactions still running and buffer
   * doesnt exceed required size already
   */
  if((!file->still_running) || (file->buffer_pos > want))
  {
//    if (!file->still_running)
//      printf("ABORT!!!!!!!!! curlm file is not running anymore!");
    //printf("ABORT - transactions not running or more chars read, %d vs %d)", file->buffer_pos, want);
    return 0;
  }

  /* attempt to fill buffer */
  do {
    int maxfd = -1;
    long curl_timeo = -1;

    FD_ZERO(&fdread);
    FD_ZERO(&fdwrite);
    FD_ZERO(&fdexcep);

    /* set a suitable timeout to fail on */
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    curl_multi_timeout(multi_handle, &curl_timeo);
    if(curl_timeo >= 0) {
      timeout.tv_sec = curl_timeo / 1000;
      if(timeout.tv_sec > 1)
        timeout.tv_sec = 1;
      else
        timeout.tv_usec = (curl_timeo % 1000) * 1000;
      //usleep(100000); //https://curl.haxx.se/libcurl/c/curl_multi_perform.html
    }

    /* get file descriptors from the transfers */
    curl_multi_fdset(multi_handle, &fdread, &fdwrite, &fdexcep, &maxfd);

    /* In a real-world program you OF COURSE check the return code of the
       function calls.  On success, the value of maxfd is guaranteed to be
       greater or equal than -1.  We call select(maxfd + 1, ...), specially
       in case of (maxfd == -1), we call select(0, ...), which is basically
       equal to sleep. */

    rc = select(maxfd+1, &fdread, &fdwrite, &fdexcep, &timeout);

    switch(rc) {
    case -1:
      /* select error */
//        printf("SELECT ERRORRRRRRRRRRRRRRRRRRRr");
      break;

    case 0:
    default:
      /* timeout or readable/writable sockets */
      curl_multi_perform(multi_handle, &file->still_running);
      break;
    }
  } while(file->still_running && (file->buffer_pos < want));
  return 1;
}

/* use to remove want bytes from the front of a files buffer */
static int use_buffer(URL_FILE *file,int want)
{
  /* sort out buffer */
  if((file->buffer_pos - want) <=0) {
    /* ditch buffer - write will recreate */
    if(file->buffer)
      free(file->buffer);

    file->buffer=NULL;
    file->buffer_pos=0;
    file->buffer_len=0;
  }
  else {
    /* move rest down make it available for later */
    memmove(file->buffer,
            &file->buffer[want],
            (file->buffer_pos - want));

    file->buffer_pos -= want;
  }
  return 0;
}

URL_FILE *url_fopen(CURLM **multi_handle, const char *url, const char *operation, long connection_timeout_ms, long request_timeout_ms)
{
  /* this code could check for URLs or types in the 'url' and
     basicly use the real fopen() for standard files */

  URL_FILE *file;
  (void)operation;

  file = (URL_FILE*)malloc(sizeof(URL_FILE));
  if(!file)
    return NULL;

  memset(file, 0, sizeof(URL_FILE));

  if((file->handle.file=fopen(url,operation)))
    file->type = CFTYPE_FILE; /* marked as URL */

  else {
    file->type = CFTYPE_CURL; /* marked as URL */
    file->handle.curl = curl_easy_init();

    curl_easy_setopt(file->handle.curl, CURLOPT_URL, url);
    curl_easy_setopt(file->handle.curl, CURLOPT_WRITEDATA, file);
    curl_easy_setopt(file->handle.curl, CURLOPT_VERBOSE, 0L);
    curl_easy_setopt(file->handle.curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(file->handle.curl, CURLOPT_CONNECTTIMEOUT_MS, connection_timeout_ms);
    curl_easy_setopt(file->handle.curl, CURLOPT_TIMEOUT_MS, request_timeout_ms);
    // Allow any authorization mode CURL knows (required for Axis MJPEG Streaming via "http://usr:pwd@host" URLs)
    // The default scheme works fine with Mobotix cameras, though!
    curl_easy_setopt(file->handle.curl, CURLOPT_HTTPAUTH, CURLAUTH_ANY);
    curl_easy_setopt(file->handle.curl, CURLOPT_TCP_KEEPALIVE, 1L);

    // To debug, set verbosity!
    //curl_easy_setopt(file->handle.curl, CURLOPT_VERBOSE, 1L);

    if(!*multi_handle)
      *multi_handle = curl_multi_init();

    curl_multi_add_handle(*multi_handle, file->handle.curl);

    // Start fetching.
    CURLMcode cc = curl_multi_perform(*multi_handle, &file->still_running);
    if(cc == CURLM_OK && (file->buffer_pos == 0) && (!file->still_running)) {
      /* if still_running is 0 now, we should return NULL */

      /* make sure the easy handle is not in the multi handle anymore */
      curl_multi_remove_handle(*multi_handle, file->handle.curl);

      /* cleanup */
      curl_easy_cleanup(file->handle.curl);

      free(file);

      file = NULL;
    }
  }
  return file;
}

int url_fclose(CURLM **multi_handle, URL_FILE *file)
{
  int ret=0;/* default is good return */

  switch(file->type) {
  case CFTYPE_FILE:
    ret=fclose(file->handle.file); /* passthrough */
    break;

  case CFTYPE_CURL:
    /* make sure the easy handle is not in the multi handle anymore */
    curl_multi_remove_handle(*multi_handle, file->handle.curl);

    /* cleanup */
    curl_easy_cleanup(file->handle.curl);
    break;

  default: /* unknown or supported type - oh dear */
    ret=EOF;
    errno=EBADF;
    break;
  }

  if(file->buffer)
    free(file->buffer);/* free any allocated buffer space */

  free(file);

  return ret;
}

int url_feof(URL_FILE *file)
{
  int ret=0;

  switch(file->type) {
  case CFTYPE_FILE:
    ret=feof(file->handle.file);
    break;

  case CFTYPE_CURL:
    if((file->buffer_pos == 0) && (!file->still_running))
      ret = 1;
    break;

  default: /* unknown or supported type - oh dear */
    ret=-1;
    errno=EBADF;
    break;
  }
  return ret;
}

size_t url_fread(CURLM *multi_handle, void *ptr, size_t size, size_t nmemb, URL_FILE *file)
{
  size_t want;

  switch(file->type) {
  case CFTYPE_FILE:
    want=fread(ptr,size,nmemb,file->handle.file);
    break;

  case CFTYPE_CURL:
    want = nmemb * size;

    fill_buffer(multi_handle, file, want);

    /* check if theres data in the buffer - if not fill_buffer()
     * either errored or EOF */
    if(!file->buffer_pos)
      return 0;

    /* ensure only available data is considered */
    if(file->buffer_pos < want)
      want = file->buffer_pos;

    /* xfer data to caller */
    memcpy(ptr, file->buffer, want);

    use_buffer(file,want);

    want = want / size;     /* number of items */
    break;

  default: /* unknown or supported type - oh dear */
    want=0;
    errno=EBADF;
    break;

  }
  return want;
}

char *url_fgets(CURLM *multi_handle, char *ptr, size_t size, URL_FILE *file)
{
  size_t want = size - 1;/* always need to leave room for zero termination */
  size_t loop;

  switch(file->type) {
  case CFTYPE_FILE:
    ptr = fgets(ptr,size,file->handle.file);
    break;

  case CFTYPE_CURL:
    fill_buffer(multi_handle, file, want);

    // check if there's data in the buffer
    if(!file->buffer_pos)
    {
//      printf("No data in the buffer!\n");
      return NULL;
    }

    // ensure only available data is considered
    if(file->buffer_pos < want)
      want = file->buffer_pos;

    /*buffer contains data */
    /* look for newline or eof */
    for(loop=0;loop < want;loop++) {
      if(file->buffer[loop] == '\n') {
        want=loop+1;/* include newline */
        break;
      }
    }

    // transfer data to caller
    memcpy(ptr, file->buffer, want);
    ptr[want]=0; // Null termination

    use_buffer(file,want);

    break;

  default: /* unknown or supported type - oh dear */
    ptr=NULL;
    errno=EBADF;
    printf("UNSUPPORTED TYPE!");
    break;
  }

  return ptr;/*success */
}

int url_http_get(const char *url, long timeout, int head_only)
{
  CURL *curl = curl_easy_init();
  CURLcode res;

  if (curl)
  {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    // Follow redirection, if we stumble upon...
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    // Set timeout in seconds
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);

    if (head_only)
      curl_easy_setopt(curl, CURLOPT_NOBODY, 1);

    // Perform GET request
    res = curl_easy_perform(curl);

    // Cleanup
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
      return res;
    else
      return 0;
  }
  return -1;
}

int url_available(const char *url, long timeout)
{
  return url_http_get(url, timeout, 1);
}

} // namespace curl
} // namespace best
} // namespace vcp
