# from flask import Flask, render_template
# app = Flask(__name__)
# @app.route('/')
# def index():
#     return render_template('index.html')


from flask import Flask, render_template
import os

PEOPLE_FOLDER = os.path.join('static', 'picture')

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = PEOPLE_FOLDER

@app.route('/')
@app.route('/index')
def show_index():
    full_filename = os.path.join(app.config['UPLOAD_FOLDER'], 'res.png')
    return render_template("index.html", user_image = full_filename)

if __name__ == '__main__':
    app.run(debug=True, port=5800, host='0.0.0.0')