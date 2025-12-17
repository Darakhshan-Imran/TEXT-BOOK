from authlib.integrations.starlette_client import OAuth
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize OAuth
oauth = OAuth()

# Register Google OAuth client
GOOGLE_CLIENT_ID = os.getenv("GOOGLE_CLIENT_ID")
GOOGLE_CLIENT_SECRET = os.getenv("GOOGLE_CLIENT_SECRET")

if GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET:
    oauth.register(
        name="google",
        client_id=GOOGLE_CLIENT_ID,
        client_secret=GOOGLE_CLIENT_SECRET,
        server_metadata_url="https://accounts.google.com/.well-known/openid_configuration",
        client_kwargs={
            "scope": "openid email profile"
        }
    )

# Register GitHub OAuth client
GITHUB_CLIENT_ID = os.getenv("GITHUB_CLIENT_ID")
GITHUB_CLIENT_SECRET = os.getenv("GITHUB_CLIENT_SECRET")

if GITHUB_CLIENT_ID and GITHUB_CLIENT_SECRET:
    oauth.register(
        name="github",
        client_id=GITHUB_CLIENT_ID,
        client_secret=GITHUB_CLIENT_SECRET,
        access_token_url="https://github.com/login/oauth/access_token",
        access_token_params=None,
        authorize_url="https://github.com/login/oauth/authorize",
        authorize_params=None,
        api_base_url="https://api.github.com/",
        client_kwargs={"scope": "user:email"},
    )