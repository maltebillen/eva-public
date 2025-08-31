#ifndef EVA_GLOBAL_EXECPTIONS_H
#define EVA_GLOBAL_EXECPTIONS_H

#include <stdexcept>


/**
 *  @class LogicError
 *
 *  @brief This class is used to throw logic errors.
 *
 *	@author Joerg Kalcsics
 *	@date 7 July 2022
 */

namespace eva {
    class LogicError : public std::logic_error
    {
    public:
        LogicError(const std::string& which, const std::string& where, const std::string& what, int err_id = 0) : std::logic_error(which + ": " + where + ":" + what) {};
        LogicError(const std::string& where, const std::string& what, int err_id = 0) : std::logic_error(std::string("LogicError") + ": " + where + ":" + what) {};
    };

    /**
     *  @class InvalidArgumentError
     *
     *  @brief This class is used to throw invalid argument errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class InvalidArgumentError : public LogicError
    {
    public:
        InvalidArgumentError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("InvalidArgumentError", where, what, err_id) {};
    };

    /**
     *  @class InvalidCallError
     *
     *  @brief This class is used to throw invalid call errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class InvalidCallError : public LogicError
    {
    public:
        InvalidCallError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("InvalidCallError", where, what, err_id) {};
    };

    /**
     *  @class CorruptedDatastructureError
     *
     *  @brief This class is used to throw data structure errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class CorruptedDatastructureError : public LogicError
    {
    public:
        CorruptedDatastructureError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("CorruptedDatastructureError", where, what, err_id) {};
    };

    /**
     *  @class SemanticError
     *
     *  @brief This class is used to throw semantic errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class SemanticError : public LogicError
    {
    public:
        SemanticError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("SemanticError", where, what, err_id) {};
    };

    /**
     *  @class DataError
     *
     *  @brief This class is used to throw data errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class DataError : public LogicError
    {
    public:
        DataError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("DataError", where, what, err_id) {};
    };

    /**
     *  @class FileError
     *
     *  @brief This class is used to throw file errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class FileError : public LogicError
    {
    public:
        FileError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("FileError", where, what, err_id) {};
    };

    /**
     *  @class MemoryError
     *
     *  @brief This class is used to throw memory errors.
     *
     *	@author Joerg Kalcsics
     *	@date 7 July 2022
     */
    class MemoryError : public LogicError
    {
    public:
        MemoryError(const std::string& where, const std::string& what, int err_id = 0) :
            LogicError("MemoryError", where, what, err_id) {};
    };
}

#endif /* EVA_EXECPTIONS_H */