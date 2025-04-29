#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rosa import ROSA
from .llm_config import get_llm
from .prompts import get_prompts
from .tools import navigation # Import navigation module
import traceback
import threading
import asyncio
import os
import pyinputplus as pyip # Using pyinputplus
from rich.console import Console
from rich.markdown import Markdown

class TurtleBotAgentNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_agent_node')
        self.get_logger().info("Initializing TurtleBot3 ROSA Agent...")
        self.rosa_agent = None
        self.console = Console()

        try:
            # Initialize shared node for tools FIRST
            navigation.initialize_node()
            self.get_logger().info("Shared node for tools initialized.")

            # --- ROSA Initialization ---
            llm = get_llm(streaming=False) # invoke is sync, so LLM streaming is false
            prompts = get_prompts()
            blacklist = ["/rosout", "/parameter_events"]

            self.rosa_agent = ROSA(
                ros_version=2,
                llm=llm,
                tool_packages=[navigation], # Pass the module
                prompts=prompts,
                blacklist=blacklist,
                verbose=True,
                accumulate_chat_history=True,
                streaming=False, # Corresponds to using invoke
            )
            # --- End ROSA Initialization ---

            # --- Define commands ---
            self.examples = [
                 "What locations can you navigate to?",
                 "Go to the kitchen",
                 "Navigate to x=1.1, y=2.6",
                 "Go to point_c",
                 "Where is the charging_station?",
                 "List ROS nodes",
            ]
            self.command_handler = {
                "help": lambda: self.submit_sync(self.get_help_text()),
                "examples": lambda: self.submit_sync(self.choose_example()),
                "clear": lambda: self.clear_sync(),
            }
            # --- End commands ---

            self.get_logger().info("ROSA Agent Initialized Successfully for CLI interaction.")

        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize ROSA Agent: {e}")
            self.get_logger().fatal(traceback.format_exc())
            if rclpy.ok(): rclpy.shutdown()
            raise

    # --- Async Helpers ---
    async def get_input_async(self, prompt: str):
        """Runs blocking pyip.inputStr in asyncio's default executor."""
        loop = asyncio.get_running_loop()
        # pyip doesn't take prompt directly, need lambda
        return await loop.run_in_executor(None, lambda: pyip.inputStr(prompt, default="help"))

    async def invoke_rosa_async(self, query: str):
        """Runs blocking rosa_agent.invoke in asyncio's default executor."""
        if not self.rosa_agent: return "Error: ROSA agent not ready."
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self.rosa_agent.invoke, query)
    # --- End Async Helpers ---

    # --- Sync Command Helpers ---
    def get_help_text(self):
         from .tools.locations import get_all_location_names # Import here to avoid circular dep.
         example_str = "\n".join([f"- {ex}" for ex in self.examples])
         commands_str = ", ".join(self.command_handler.keys())
         locations_str = ", ".join(get_all_location_names())
         return f"""
## TurtleBot3 ROSA Agent Help

You can ask me to navigate using coordinates (e.g., 'go to x=1, y=2, yaw_degrees=90') or named locations.

**Available Commands:**
- `{commands_str}`
- `exit`: Quit the agent.

**Named Locations:**
- {locations_str}

**Example Queries:**
{example_str}
         """

    def choose_example(self):
        """Gets user selection from examples (sync)."""
        try:
            # Use console for potentially better interaction than raw pyip
            self.console.print("\nAvailable Examples:")
            choice = pyip.inputMenu(
                self.examples,
                prompt="Enter example choice number:\n",
                numbered=True,
                blank=False,
                timeout=60,
                default=self.examples[0] # Default to first example
            )
            return choice
        except pyip.TimeoutException:
            self.console.print("[yellow]Example selection timed out.[/yellow]")
            return None
        except Exception as e:
            self.console.print(f"[red]Error selecting example: {e}[/red]")
            return None

    def clear_sync(self):
        """Clears chat history and console (sync)."""
        if self.rosa_agent: self.rosa_agent.clear_chat()
        os.system('clear')
        self.console.print("[bold green]Chat history cleared.[/bold green]")
        return None # Signal not to process further

    def submit_sync(self, query: str):
         """Helper to submit a query synchronously for command_handler results."""
         if not query: return
         self.console.print(f"[blue]Processing command: '{query}'...[blue]")
         if self.rosa_agent:
              response = self.rosa_agent.invoke(query)
              self.console.print(Markdown(response))
         else:
              self.console.print("[bold red]Error: ROSA agent not initialized.[/bold red]")
    # --- End Sync Command Helpers ---

    # --- Main Async Interaction Loop ---
    async def run(self):
        """Runs the main asynchronous interaction loop."""
        self.console.print("[bold green]TurtleBot3 ROSA Agent Ready.[/bold green]")
        self.console.print("Type command, 'help', 'examples', 'clear', or 'exit'.")

        while rclpy.ok():
            try:
                self.get_logger().info("Waiting for user input (async)...")
                # Use the async helper for input
                user_input = await self.get_input_async("> ")
                self.get_logger().info(f"Received input: '{user_input}'")

                if not user_input: continue

                if user_input.lower() == "exit":
                    self.get_logger().info("Exit command received.")
                    break
                elif user_input.lower() in self.command_handler:
                    # Execute sync command handler directly
                    self.command_handler[user_input.lower()]()
                    continue

                # Process regular query with ROSA asynchronously
                self.console.print(f"[blue]Processing query: '{user_input}'...[blue]")
                response = await self.invoke_rosa_async(user_input)
                self.console.print(Markdown(response)) # Display response

            except (KeyboardInterrupt, EOFError):
                self.get_logger().info("Interrupt or EOF detected, exiting loop.")
                break
            except Exception as e:
                self.console.print(f"[bold red]An error occurred in the input loop: {e}[bold red]")
                self.get_logger().error(f"Error in async run loop: {e}")
                self.get_logger().error(traceback.format_exc())

        self.get_logger().info("Exiting async run loop.")
    # --- End Main Async Loop ---


def main(args=None):
    rclpy.init(args=args)
    agent_node = None
    shared_tools_node = None
    executor = None
    executor_thread = None

    try:
        agent_node = TurtleBotAgentNode()
        # Critical: Get the node *after* it's created in agent_node's init
        shared_tools_node = navigation.get_shared_node()
        if shared_tools_node is None: raise RuntimeError("Shared tools node failed to initialize.")

        executor = MultiThreadedExecutor()
        executor.add_node(agent_node)
        executor.add_node(shared_tools_node)

        print("Starting background executor thread...")
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        print("Starting asyncio event loop for CLI...")
        asyncio.run(agent_node.run()) # Runs the async def run method

    except KeyboardInterrupt:
        print("\nCtrl+C detected in main, initiating shutdown...")
    except Exception as e:
         logger = agent_node.get_logger() if agent_node else rclpy.logging.get_logger('turtlebot3_agent_main')
         logger.fatal(f"Agent node encountered an error: {e}")
         logger.fatal(traceback.format_exc())
    finally:
        # Cleanup
        print("Shutting down executor, nodes, and rclpy...")
        if executor:
             print("Cancelling executor tasks...")
             executor.shutdown() # Waits for tasks
        if agent_node:
             print("Destroying main agent node...")
             agent_node.destroy_node()
        if shared_tools_node:
             print("Destroying shared tools node...")
             shared_tools_node.destroy_node()
        if rclpy.ok():
             print("Shutting down rclpy...")
             rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()