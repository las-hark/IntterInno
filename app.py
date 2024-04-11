from flask import Flask, request, send_from_directory, jsonify
import os

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# 确保上传文件夹存在
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/upload', methods=['POST'])
def upload_file():
    #print(request.headers)
    filename = dict(request.headers)['Device'] + ".wav"
    print(filename)
    file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)

    with open(file_path, "wb") as file:
        file.write(request.data)
    # print(request.data)
    # if 'file' not in request.files:
    #     return jsonify({"message": "NO FILE"}), 404
    # file = request.files['file']
    # if file.filename == '':
    #     return jsonify({"error": "No selected file"}), 402
    # if file:
    #     filename = file.filename
    #     file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
    return jsonify({
        "message": "文件上传成功",
        "url": f"http://localhost:5000/uploads/{filename}"
    }), 201


@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)


@app.route('/')
def index():
    return 'Hello, World!'



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
