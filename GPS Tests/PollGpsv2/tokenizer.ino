
//-------------------------------------------------
// Function: myStrtok()
// Purpose; Split a string passed in into tokens
//        using a given string of characters as the
//        split characters.
// Args: parseStr - String to be parsed or NULL if
//            parsing is to continue with the current
//            string.
//       splitChars - Array of characters to use as
//            the split characters.
// Returns: Pointer to next token in the string
//--------------------------------------------------
char *myStrtok(char *parseStr, char *splitChars)
{
    // Note: The first var is declared as "static" this
    //  means that after the initial call to the function it
    //    will retain its last value with each subsequent call
    //    to this function. It is initialized on the very first
    //    call to this function to NULL.
    static char *pStr = NULL; // Pointer to string to be tokenized.

    char *tok;  // Pointer to start of next token
    char *temp; // Misc. use temporary pointer.
    int  found; // Boolean flag to indicate a split character was found

    //----------------------------------------------------------------
    // Case 1: See if a new string to be parsed was passed in.  This
    //            will be true if parseStr is not NULL
    //----------------------------------------------------------------
    if(parseStr != NULL)
        pStr = parseStr; // If yes, then hold the pointer to it

    //----------------------------------------------------------------
    // Case 2: Check to see if the last call to this function returned 
    //            the last token in the string. If pStr is now pointing 
    //            to the NULL terminator this will be true
    //----------------------------------------------------------------
    if(*pStr == '\0') return NULL; // Tell user all tokens have been returned

    //----------------------------------------------------------------
    // Case 3: Find the next token.  
    //        Step 1: Starting from the end of the last token returned
    //            or the beginning of the new string to parse skip any
    //            leading characters that are the same as characters found
    //            in the splitChars array. We also look for the NULL
    //            terminator indicating we have reached the end of parseStr.
    //----------------------------------------------------------------
    found = 0; // Initialize to FALSE
    tok = pStr; // Initialize tok pointer to start current point in parseStr
    // Skip any leading splitChars
    while((!found) && (*tok != '\0'))
    {
        temp = splitChars;    // Point to start of splitChars array
        while(*temp != '\0') // Scan entire splitChars array each time
        {
            if(*tok != *temp) 
            {
                temp++; // Advance to next character in splitChars
            }
            else    // Found a split char
            {
                tok++; // Advance to next character in parseStr
                break; // and end this scan of the splitChars array
            }
        }
        // Check to see if we made it through the entire splitChars
        // array without finding a match, i.e. we have the first char
        // in the next token
        if(*temp == '\0') found = 1; // Mark as TRUE to end search
        // Note: If tok was advanced to point to the NULL terminator at the
        //   end of parseStr this will also terminate the loop
    }

    // Check to see if we reached the end of parseStr without finding another
    //        token.  If so set pStr so we can recognize this at the next call
    if(*tok == '\0')
    {
        pStr = tok; // Point pStr to the NULL terminator at the end of parseStr
        return tok; // Return NULL to indicate the end of the string was reached.
    }

    // When we reach this point tok points to the first non-splitChars character

    //----------------------------------------------------------------
    //        Step 2: Find the end of this token.  This will be the next
    //            occurance of one of the characters in splitChars or the
    //            NULL terminator marking the end of parseStr
    //----------------------------------------------------------------
    found = 0; // Initialize to FALSE
    pStr = tok; // Initialize pStr to tok
    // Search for first occurance of a splitChars character marking the end
    //    of this token.  Also look to see if we reach the end of parseStr
    while((!found) && (*pStr != '\0'))
    {
        temp = splitChars;    // Point to start of splitChars array
        // Scan entire splitChars array to see if the char pStr points to
        // is one of the split chars.
        while((*temp != '\0') && (*pStr != *temp)) temp++;

        // if this char was OK advance to the next and try again
        if(*temp == '\0') 
            pStr++;
        else
            found = 1;    // Found the end of the token so end the while() loop
        // Note: If pStr was advanced to point to the NULL terminator at the
        //   end of parseStr this will also terminate the loop
    }

    // At this point we have tok pointing to the first character of the
    //    next token in parseStr and pStr pointing to the first character
    //    after the end of the next token.

    //----------------------------------------------------------------
    //        Step 3: Set up for the return and next call.
    //----------------------------------------------------------------
    // When we reach this point if pStr is pointing to the NULL terminator
    //    at the end of parseStr we leave pStr pointing to this NULL terminator
    //    so we will know this on the next call to this function. 

    if(*pStr != '\0')
    {
        // However, if pStr is not pointing to a NULL terminator then it
        //    must be pointing to a split character so we replace the character 
        //    pStr is pointing to with a NULL terminator so the caller can get 
        //    the token by itself and advance pStr to the first character after 
        //    that so it is ready to parse the next token on the next call to 
        //    this function.
        *pStr = '\0'; // Put a NULL terminator at the end of this token
        pStr++; // Advance pStr to the next character in parseStr
    }

    return 0; // Return the pointer to the next token
}

