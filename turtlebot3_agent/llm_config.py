import os
import dotenv
from langchain_openai import ChatOpenAI # Or AzureChatOpenAI, ChatOllama

def get_llm(streaming: bool = False):
    """Gets the LLM instance."""
    # Load environment variables (expects .env file in workspace root: ~/ros2_ws/.env)
    dotenv.load_dotenv(dotenv.find_dotenv(filename='.env', raise_error_if_not_found=False)) # Non-critical if not found initially
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        # Provide a more helpful error message
        print("\n" + "="*30 + " ERROR " + "="*30)
        print(" OpenAI API Key not found! ")
        print(" Please create a '.env' file in your '~/ros2_ws' directory")
        print(" and add the line: OPENAI_API_KEY=your_actual_key_here")
        print("="*67 + "\n")
        raise ValueError("OPENAI_API_KEY not found in environment variables or .env file.")

    # Recommended model: gpt-4o or gpt-4o-mini
    llm = ChatOpenAI(
        model_name="gpt-4o-mini",
        openai_api_key=api_key,
        temperature=0, # Low temperature for more predictable control commands
        streaming=streaming,
        # Add timeout if needed
        # request_timeout=60,
    )
    print(f"LLM initialized (model: gpt-4o-mini, streaming: {streaming})")
    return llm

def get_env_variable(var_name: str) -> str:
    """Safely retrieves environment variables."""
    # Ensure .env is loaded if needed
    dotenv.load_dotenv(dotenv.find_dotenv(filename='.env', raise_error_if_not_found=False))
    value = os.getenv(var_name)
    if value is None:
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value