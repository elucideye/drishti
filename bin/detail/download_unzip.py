import os
import requests
import subprocess

# http://stackoverflow.com/a/16696317/2288008
def file_download_once(url, local_path):
  print('Downloading:\n  {}\n  -> {}'.format(url, local_path))
  r = requests.get(url, stream=True)
  if not r.ok:
    raise Exception('Downloading failed: {}'.format(url))
  with open(local_path, 'wb') as f:
    for chunk in r.iter_content(chunk_size=16*1024):
      if chunk:
        f.write(chunk)

def file_download(url, local_path):
  max_retry = 3
  for i in range(max_retry):
    try:
      file_download_once(url, local_path)
      print('Done')
      return
    except Exception as exc:
      print('Exception catched ({}), retry... ({} of {})'.format(exc, i+1, max_retry))
      time.sleep(60)
  sys.exit('Download failed')

def download_unzip(url, todir):
  if not os.path.exists(todir):
    os.mkdir(todir)
  os.chdir(todir)

  temp_local_path = os.path.join(os.getcwd(), 'temp.zip')

  file_download(url, temp_local_path)

  # Can't use ZipFile module because permissions will be lost, see bug:
  # * https://bugs.python.org/issue15795
  devnull = open(os.devnull, 'w') # subprocess.DEVNULL is not available for Python 3.2
  subprocess.check_call(
      ['7z', 'x', '-y', temp_local_path], stdout=devnull, stderr=devnull, bufsize=0
  )

  os.remove(temp_local_path)
  os.chdir('..')
