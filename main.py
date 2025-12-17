from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import API routers
from src.api.auth import router as auth_router
from src.api.users import router as user_router
from src.api.protected import router as protected_router
from src.database.connection import init_db

# Create FastAPI application
app = FastAPI(
    title="Physical AI & Humanoid Robotics - Auth API",
    description="Authentication API for the educational platform",
    version="1.0.0"
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://darakhshan-imran.github.io",  # GitHub Pages production
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose authorization header to allow client to access token
    expose_headers=["Authorization"]
)


# Event handlers
@app.on_event("startup")
async def startup_event():
    """
    Initialize the database when the application starts.
    This function is called automatically when the application starts.
    """
    init_db()


# Include API routers
app.include_router(auth_router, prefix="/auth", tags=["authentication"])
app.include_router(user_router, prefix="/users", tags=["users"])
app.include_router(protected_router, prefix="/protected", tags=["protected"])


# Root endpoint
@app.get("/")
async def root():
    """
    Root endpoint to check if the API is running.
    """
    return {"message": "Authentication API is running!"}


# Error handlers
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    """
    Handle validation errors and return a proper error response.
    """
    return JSONResponse(
        status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
        content={
            "detail": "Validation error",
            "errors": [
                {
                    "loc": error["loc"],
                    "msg": error["msg"],
                    "type": error["type"]
                }
                for error in exc.errors()
            ]
        }
    )


@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """
    Handle HTTP exceptions and return a proper error response.
    """
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "detail": exc.detail
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """
    Handle general exceptions and return a proper error response.
    """
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "detail": "Internal server error"
        }
    )