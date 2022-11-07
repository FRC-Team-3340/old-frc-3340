// This is a test program for us to learn how to use the repository. 
// Goal: be able to push, pull, and commit changes to the GitHub repository.
public class PracticeGit{
    // Note this code is from W3Schools, in case you want to learn Java on your own time.
    public static void main(String[] args) {
        // TODO: add your own print statement following this syntax here.
        // Follow appropriate Java syntax. The code here is exactly how it should be.
        // Replace the message with your own and sign with your name at the end.

        System.out.println("Hello, world! -> Ryan");
        System.out.println("williams doesnt know how to download on windows");

        // Your messages:
<<<<<<< HEAD
        System.out.println("Hello guys! -> anderson with the huge bvalls");

=======
        System.out.println("Hello guys! -> Ismael");
        System.out.println("I do not know what i'm doing! -> williams");
>>>>>>> 800d7bff243b024919e02579987fef342abdd6c2
    }
}

// Once your message is done, commit to main and push to origin either via the Terminal or GitHub Desktop.
// In the terminal, begin by typing "git add ."
// Then, type git commit -m "(describe your commit here)".
// Finally, type "git push".
// Check out your other members' messages by typing "git pull" every so often. If any issues arise, such as merge conflicts, please let me know.