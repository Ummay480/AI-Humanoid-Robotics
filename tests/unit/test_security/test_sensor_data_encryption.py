"""
Unit tests for the sensor data encryption module.
"""
import json
import pytest
from src.perception.security.sensor_data_encryption import SensorDataEncryption, create_sensor_data_signature, verify_sensor_data_signature


class TestSensorDataEncryption:
    """Test sensor data encryption functionality."""

    def test_initialization_with_password(self):
        """Test initializing with a password."""
        encryptor = SensorDataEncryption(password="test_password")
        assert encryptor.key is not None
        assert len(encryptor.key) > 0

    def test_initialization_without_password(self):
        """Test initializing without a password."""
        encryptor = SensorDataEncryption()
        assert encryptor.key is not None
        assert len(encryptor.key) > 0

    def test_encrypt_decrypt_string(self):
        """Test encrypting and decrypting a string."""
        encryptor = SensorDataEncryption(password="test_password")
        original_data = "Hello, sensor data!"

        encrypted = encryptor.encrypt_sensor_data(original_data)
        decrypted = encryptor.decrypt_sensor_data(encrypted)

        assert decrypted == original_data

    def test_encrypt_decrypt_dict(self):
        """Test encrypting and decrypting a dictionary."""
        encryptor = SensorDataEncryption(password="test_password")
        original_data = {
            "sensor_id": "camera_front",
            "timestamp": 1234567890.123,
            "data": [1, 2, 3, 4, 5],
            "metadata": {"resolution": "1920x1080", "format": "RGB"}
        }

        encrypted = encryptor.encrypt_sensor_data(original_data)
        decrypted = encryptor.decrypt_sensor_data(encrypted)

        assert decrypted == original_data

    def test_encrypt_decrypt_bytes(self):
        """Test encrypting and decrypting bytes."""
        encryptor = SensorDataEncryption(password="test_password")
        original_data = b"binary sensor data"

        encrypted = encryptor.encrypt_sensor_data(original_data)
        decrypted = encryptor.decrypt_sensor_data(encrypted)

        assert decrypted == original_data.decode('utf-8')

    def test_hash_sensor_data(self):
        """Test hashing sensor data."""
        encryptor = SensorDataEncryption()
        data = {"sensor_id": "test", "value": 42}

        hash1 = encryptor.hash_sensor_data(data)
        hash2 = encryptor.hash_sensor_data(data)

        # Same data should produce same hash
        assert hash1 == hash2

        # Different data should produce different hash
        different_data = {"sensor_id": "test", "value": 43}
        hash3 = encryptor.hash_sensor_data(different_data)
        assert hash1 != hash3

    def test_verify_data_integrity(self):
        """Test verifying data integrity."""
        encryptor = SensorDataEncryption()
        data = {"sensor_id": "test", "value": 42}

        data_hash = encryptor.hash_sensor_data(data)
        assert encryptor.verify_data_integrity(data, data_hash) is True

        # Different data should fail integrity check
        different_data = {"sensor_id": "test", "value": 43}
        assert encryptor.verify_data_integrity(different_data, data_hash) is False

    def test_encrypt_decrypt_sensor_payload(self):
        """Test encrypting and decrypting sensor payload with metadata."""
        encryptor = SensorDataEncryption(password="test_password")
        sensor_id = "lidar_3d"
        payload = {
            "sensor_id": sensor_id,
            "timestamp": 1234567890.123,
            "ranges": [1.0, 2.0, 3.0, 4.0, 5.0]
        }

        encrypted_package = encryptor.encrypt_sensor_payload(sensor_id, payload)
        decrypted_payload = encryptor.decrypt_sensor_payload(encrypted_package)

        assert decrypted_payload == payload

    def test_invalid_encryption_method(self):
        """Test decryption with invalid encryption method."""
        encryptor = SensorDataEncryption()

        with pytest.raises(ValueError):
            encryptor.decrypt_sensor_data({
                'encrypted_data': 'some_data',
                'encryption_method': 'invalid_method'
            })

    def test_missing_encrypted_data_field(self):
        """Test decryption with missing encrypted_data field."""
        encryptor = SensorDataEncryption()

        with pytest.raises(ValueError):
            encryptor.decrypt_sensor_data({
                'encryption_method': 'Fernet'
            })

    def test_unsupported_data_type_for_encryption(self):
        """Test encryption with unsupported data type."""
        encryptor = SensorDataEncryption()

        with pytest.raises(ValueError):
            encryptor.encrypt_sensor_data(123)  # Integer not supported

    def test_unsupported_data_type_for_hashing(self):
        """Test hashing with unsupported data type."""
        encryptor = SensorDataEncryption()

        with pytest.raises(ValueError):
            encryptor.hash_sensor_data(123)  # Integer not supported


class TestDigitalSignatures:
    """Test digital signature functionality."""

    def test_create_and_verify_signature_dict(self):
        """Test creating and verifying signature for dictionary data."""
        data = {"sensor_id": "camera_front", "value": 42}

        signature = create_sensor_data_signature(data)
        assert verify_sensor_data_signature(data, signature) is True

        # Different data should fail verification
        different_data = {"sensor_id": "camera_front", "value": 43}
        assert verify_sensor_data_signature(different_data, signature) is False

    def test_create_and_verify_signature_string(self):
        """Test creating and verifying signature for string data."""
        data = "sensor data string"

        signature = create_sensor_data_signature(data)
        assert verify_sensor_data_signature(data, signature) is True

        # Different data should fail verification
        different_data = "different sensor data"
        assert verify_sensor_data_signature(different_data, signature) is False