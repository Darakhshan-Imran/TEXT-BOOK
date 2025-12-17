import os
import sys
from dotenv import load_dotenv

# Add the project root to the path so we can import modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
load_dotenv()

from src.database.connection import SessionLocal
from src.auth.services import register_user
from src.schemas.auth import UserCreate

def test_registration():
    # Create a test user
    db = SessionLocal()
    try:
        user_data = UserCreate(
            email='testuser@example.com',
            password='Testpassword123!',
            first_name='Test',
            last_name='User'
        )
        user = register_user(db, user_data)
        print(f'User created successfully: {user.email}')
        print(f'User ID: {user.id}')
        print(f'User UUID: {user.uuid}')
    except Exception as e:
        print(f'Error creating user: {e}')
        import traceback
        traceback.print_exc()
    finally:
        db.close()

if __name__ == "__main__":
    test_registration()