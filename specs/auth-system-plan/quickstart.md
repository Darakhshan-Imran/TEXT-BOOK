# Quickstart Guide: Authentication System Implementation

## Prerequisites

- Python 3.8 or higher
- pip package manager
- Git
- Access to Neon Database (PostgreSQL)
- OAuth2 credentials for Google and GitHub

## Setup Instructions

### 1. Clone and Navigate to Project

```bash
git clone <your-repo-url>
cd <project-directory>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install fastapi uvicorn sqlalchemy psycopg2-binary python-jwt passlib[bcrypt] authlib python-multipart python-dotenv
```

### 4. Set Up Environment Variables

Create a `.env` file in your project root:

```env
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname
SECRET_KEY=your-super-secret-and-long-random-string-here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
FRONTEND_URL=http://localhost:3000  # Your frontend URL for OAuth redirects
```

### 5. Project Structure

Create the following directory structure:

```
src/
├── __init__.py
├── database/
│   ├── __init__.py
│   ├── models.py
│   ├── connection.py
│   └── session.py
├── auth/
│   ├── __init__.py
│   ├── jwt.py
│   ├── password.py
│   ├── oauth.py
│   ├── services.py
│   └── dependencies.py
├── api/
│   ├── __init__.py
│   ├── auth.py
│   ├── users.py
│   └── protected.py
└── utils/
    ├── __init__.py
    └── validators.py
```

## Implementation Steps

### Step 1: Database Models

Create the database models in `src/database/models.py`:

```python
from sqlalchemy import Boolean, Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    uuid = Column(String, unique=True, index=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=True)  # Null for OAuth users
    first_name = Column(String, nullable=True)
    last_name = Column(String, nullable=True)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    auth_provider = Column(String, nullable=False)  # 'email', 'google', 'github'
    provider_id = Column(String, nullable=True)  # For OAuth users
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
```

### Step 2: Database Connection

Create database connection in `src/database/connection.py`:

```python
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from .models import Base
import os
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

def init_db():
    Base.metadata.create_all(bind=engine)
```

### Step 3: JWT Utilities

Create JWT utilities in `src/auth/jwt.py`:

```python
from datetime import datetime, timedelta
from typing import Optional
import jwt
from passlib.context import CryptContext
from fastapi import HTTPException, status
from dotenv import load_dotenv
import os

load_dotenv()

SECRET_KEY = os.getenv("SECRET_KEY")
ALGORITHM = os.getenv("ALGORITHM")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", 30))

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_uuid: str = payload.get("sub")
        if user_uuid is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return user_uuid
    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
```

### Step 4: Main Application

Update your `main.py` to include the authentication system:

```python
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from src.api.auth import router as auth_router
from src.api.users import router as user_router
from src.api.protected import router as protected_router
from src.database.connection import init_db

app = FastAPI(title="Physical AI & Humanoid Robotics - Auth API")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure properly for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database
@app.on_event("startup")
def on_startup():
    init_db()

# Include routers
app.include_router(auth_router, prefix="/auth", tags=["authentication"])
app.include_router(user_router, prefix="/users", tags=["users"])
app.include_router(protected_router, prefix="/protected", tags=["protected"])

@app.get("/")
def read_root():
    return {"message": "Authentication API is running!"}
```

## Running the Application

```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

## API Endpoints

- `POST /auth/register` - Register new user
- `POST /auth/login` - Login with email/password
- `GET /auth/google` - Initiate Google OAuth
- `GET /auth/github` - Initiate GitHub OAuth
- `GET /auth/profile` - Get user profile (requires auth)
- `GET /protected/resource` - Access protected resource (requires auth)

## Testing OAuth Setup

For Google OAuth:
1. Go to Google Cloud Console
2. Create OAuth 2.0 credentials
3. Add `http://localhost:8000/auth/google/callback` to authorized redirect URIs

For GitHub OAuth:
1. Go to GitHub Developer Settings
2. Create OAuth App
3. Add `http://localhost:8000/auth/github/callback` as callback URL