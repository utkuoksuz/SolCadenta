import http.server
import os

PORT = 8080
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

print(f"Serving at http://localhost:{PORT}")
http.server.HTTPServer(("", PORT), Handler).serve_forever()
