from app.app import db


class MotorPosition(db.Model):

    __tablename__ = "motor_position"

    id = db.Column(db.Integer, primary_key=True)
    position = db.Column(db.Integer, nullable=False)
    motorname = db.Column(db.String, nullable=False) # TODO: add foreign key constraint
    pose_id = db.Column(db.Integer, db.ForeignKey('pose.id'), nullable=False)
