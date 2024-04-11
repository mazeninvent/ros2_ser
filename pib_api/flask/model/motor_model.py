from app.app import db
from model.bricklet_pin_model import BrickletPin


class Motor(db.Model):
    __tablename__ = "motor"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(255), nullable=False, unique=True)
    pulse_width_min = db.Column("pulseWidthMin", db.Integer, nullable=False)
    pulse_width_max = db.Column("pulseWidthMax", db.Integer, nullable=False)
    rotation_range_min = db.Column("rotationRangeMin", db.Integer, nullable=False)
    rotation_range_max = db.Column("rotationRangeMax", db.Integer, nullable=False)
    velocity = db.Column(db.Integer, nullable=False)
    acceleration = db.Column(db.Integer, nullable=False)
    deceleration = db.Column(db.Integer, nullable=False)
    period = db.Column(db.Integer, nullable=False)
    turned_on = db.Column("turnedOn", db.Boolean, nullable=False)
    visible = db.Column(db.Boolean, nullable=False)
    invert = db.Column(db.Boolean, nullable=False)
    bricklet_pins = db.relationship("BrickletPin", backref="motor", lazy=True, cascade="all,, delete-orphan")
