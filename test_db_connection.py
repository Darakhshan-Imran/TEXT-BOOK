import os
import sys
from dotenv import load_dotenv

# Add the project root to the path so we can import modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

load_dotenv()

try:
    from src.database.connection import engine
    from sqlalchemy import text

    # Test the database connection
    with engine.connect() as connection:
        result = connection.execute(text("SELECT 1"))
        print("Database connection successful!")
        print("Result:", result.fetchone())

except Exception as e:
    print(f"Database connection failed: {e}")
    import traceback
    traceback.print_exc()