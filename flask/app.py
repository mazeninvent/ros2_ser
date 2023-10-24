import os
import uuid
from flask import Flask, jsonify, request
from flask_sqlalchemy import SQLAlchemy
from flask_marshmallow import Marshmallow

app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname("/home/pib/pib_data/"))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'pibdata.db')
db = SQLAlchemy(app)
ma = Marshmallow(app)

class Personality(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False)
    personality_id = db.Column(db.String(255), nullable=False)
    gender = db.Column(db.String(255), nullable=False)
    description = db.Column(db.String(38000), nullable=True)
    pause_threshold = db.Column(db.Numeric, nullable=False)
    def __init__(self, *args):
        if len(args) == 3:
            self.eq3(args)
        if len(args) == 5:
            self.eq5(args)

    def eq3(self, args):
        self.name = args[0]
        self.personality_id = str(uuid.uuid4())
        self.description = ""
        self.gender = args[1]
        self.pause_threshold = args[2]

    def eq5(self, args):
        self.name = args[0]
        self.personality_id = args[1]
        self.gender = args[2]
        self.description = args[3]
        self.pause_threshold = args[4]

class CameraSettings(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    resolution = db.Column(db.String(20), nullable=False)
    refrash_rate = db.Column(db.Float, nullable=False)
    quality_factor = db.Column(db.Integer, nullable=False)
    is_active = db.Column(db.Boolean, nullable=False)

    def __init__(self, resolution, refrash_rate, quality_factor, is_active):
        self.resolution = resolution
        self.refrash_rate = refrash_rate
        self.quality_factor = quality_factor
        self.is_active = is_active


class PersonalitySchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Personality
        exclude = ('id',)

class CameraSettingsSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = CameraSettings
        exclude = ('id',)

personality_schema = PersonalitySchema()
personalities_schema = PersonalitySchema(many=True)
camera_settings_schema = CameraSettingsSchema()

@app.route('/voice-assistant/personality')
def get_all_personalities():
    all_personalities = Personality.query.all()
    return jsonify({"voiceAssistantPersonalities": personalities_schema.dump(all_personalities)})

@app.route('/voice-assistant/personality/<string:uuid>', methods=['GET'])
def get_personality_by_id(uuid):
    getPersonality = Personality.query.filter(Personality.personality_id == uuid).first()
    return personality_schema.jsonify(getPersonality)

@app.route('/voice-assistant/personality', methods=['POST'])
def create_personality():
    personality = Personality(request.json.get('name'), request.json.get('gender'), request.json.get('pause_threshold'))
    db.session.add(personality)
    db.session.commit()
    returnPersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    return jsonify(personality_schema.dump(returnPersonality)), 201

@app.route('/voice-assistant/personality/<string:uuid>', methods=['PUT'])
def update_personality(uuid):
    personality = Personality(request.json.get('name'), uuid, request.json.get('gender'), request.json.get('description'), request.json.get('pause_threshold'))
    updatePersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    updatePersonality.name = personality.name
    updatePersonality.description = personality.description
    updatePersonality.gender = personality.gender
    updatePersonality.pause_threshold = personality.pause_threshold
    db.session.add(updatePersonality)
    db.session.commit()
    updatePersonality = Personality.query.filter(Personality.personality_id == personality.personality_id).first()
    return personality_schema.jsonify(updatePersonality)

@app.route('/voice-assistant/personality/<string:uuid>', methods=['DELETE'])
def delete_personality(uuid):
    deletePersonality = Personality.query.filter(Personality.personality_id == uuid).first()
    db.session.delete(deletePersonality)
    db.session.commit()
    return '', 204

@app.route('/camera-settings', methods=['GET'])
def get_camera_settings():
    cameraSettings = CameraSettings.query.all()
    return camera_settings_schema.jsonify(cameraSettings[0])


@app.route('/camera-settings', methods=['PUT'])
def update_camera_settings():
    newCameraSettings = CameraSettings(request.json.get('resolution'), request.json.get('refrash_rate'), request.json.get('quality_factor'), request.json.get('is_active'))
    updateCameraSettings = CameraSettings.query.filter(CameraSettings.id == 0).first()
    updateCameraSettings.resolution = newCameraSettings.resolution
    updateCameraSettings.refresh_rate = newCameraSettings.refrash_rate
    updateCameraSettings.quality_factor = newCameraSettings.quality_factor
    updateCameraSettings.is_active = newCameraSettings.is_active
    db.session.add(updateCameraSettings)
    db.session.commit()
    response = CameraSettings.query.filter(CameraSettings.id == 0).first()
    print(response.is_active)
    return camera_settings_schema.jsonify(response)

if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")