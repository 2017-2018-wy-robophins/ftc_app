package org.firstinspires.ftc.teamcode.autonomous.commands;

public class CommandTreeBuilder {
    //A structure that allows for simple creation of asynchronous commands.
    private CommandTree commandTree = new CommandTree();

    public CommandTreeBuilder(Command command) {
        commandTree = new CommandTree(command);
    }

    public CommandTreeBuilder addChildCommand(CommandTree child) {
        commandTree.children.add(child);
        return this;
    }

    public CommandTree create() {
        return commandTree;
    }
}
