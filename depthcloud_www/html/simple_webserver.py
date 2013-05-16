#!/usr/bin/python

from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from os import path
import rospkg
import subprocess
import sys

rp = rospkg.RosPack()

class MyHandler(BaseHTTPRequestHandler):            
    def do_GET(self):
        print "\nIncoming request!"
        try:
            # Get resources from other packages (meshes)
            if self.path.startswith("/resources/"):
                splitPath = self.path.split("/")
                if len(splitPath) > 2:
                    splitPath.remove('');
                    try:
                        pkg_path = rp.get_path(splitPath[1])
                    except rospkg.ResourceNotFound:
                        self.send_error(404, "Resource not found: %s" % splitPath[1])
                        return

                    subdir = self.path.replace('/resources/' + splitPath[1], '')
                    filePath = pkg_path + subdir

                    fileHead, fileTail = path.split(filePath)
                    fileName, fileExtension = path.splitext(fileTail);

                    print fileName;
                    print fileExtension;
                    
                    print "opening ", filePath
                    # convert tif to png 
                    # (needs ImageMagick. PIL can't read some of the PR2 tiff files)
                    if fileExtension == '.tif':
                        pngFile = '/tmp/' + fileName + '.png'
                        print pngFile
                        
                        from subprocess import call
                        call(["convert", filePath, pngFile ])
                        
                        filePath = pngFile

                    try:
                        f = open(filePath)
                    except:
                        self.send_error(404, 'Requested file not found: %s' % self.path)
                        return

                    self.send_response(200)
                    self.send_header('Content-Length', path.getsize(filePath))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(f.read())
                    
                    f.close()
                    return
                else:
                    self.send_error(404, 'Not enough information supplied: %s' % self.path)
            # Get files from the local file system
            else:
                filePath = "."+self.path
                print 'Getting', filePath
                try:
                    f = open(filePath) 
                except:
                    self.send_error(404, 'Requested file not found: %s' % filePath)
                    return

                self.send_response(200)
                #self.send_header('Content-Type', "text/html")
                self.send_header('Content-Length', path.getsize(filePath))
                self.send_header('Access-Control-Allow-Origin', '*')
#                self.send_header('Content-Security-Policy', 'img-src \'http://localhost:8888\'; connect-src \'http://localhost:8888\'; video-src \'http://localhost:8888\'; frame-src \'self\'')
                self.send_header('Access-Control-Allow-Methods', '*')
#                self.send_header('Access-Control-Allow-Headers', 'video/webm')
                
                self.end_headers()
                self.wfile.write(f.read())
                f.close()
                return
        except IOError:
            self.send_error(404, 'File Not Found: %s' % self.path)

def main():
    try:
        port = 8000
        server = HTTPServer(('', port), MyHandler)
        print '\nStarted webserver on port', port
        server.serve_forever()
    except KeyboardInterrupt:
        print 'Control-C received, shutting down server'
        server.socket.close()

if __name__ == '__main__':
    main()
